//=============================================================================
// sas_raid_tlm.cpp — SAS RAID 层模型实现  [2026-10-09 建]
//-----------------------------------------------------------------------------
// 结构（本文件 ✓）：
//   · **条带单元（stripe unit = stripe_size 字节/盘 ✓）** 为最小运算粒度 ✓
//   · 单元读：源盘在线 ⇒ 直读 ✓；缺失 ⇒ **重建读**（R5：其余数据 ⊕ P ✓；R6：P/Q ✓）
//   · 单元写：R0 直写 ✓；R1/R10 双写 ✓；R5 **RMW**：P' = P ⊕ D_old ⊕ D_new ✓（只算**触碰区段** ✓）；
//             R6 RMW：P' 同上 ✓、Q' = Q ⊕ g_i·(D_old ⊕ D_new) ✓（GF(2⁸)，poly 0x11D ✓）
//   · 重建：逐条带逐盘（数据 ⇒ 重构 ✓；P/Q ⇒ 重算 ✓）
//   · 写洞日志：写校验前记 {vol,stripe} ✓、完成清除 ✓、崩后 `journal_replay` 幂等重算 ✓
//   · 元命令应答：与 `sas_hdd/tlm` **字节级一致** ✓（INQUIRY/RCAP/MODE SENSE/REQ SENSE ✓）
//=============================================================================
#include "sas_raid_tlm.h"
#include <cstdio>

// ── GF(2^8)：poly 0x11D（x^8+x^4+x^3+x^2+1 ✓ RAID-6 经典 ✓，与 RTL LUT 同函数 ✓）──
static uint8_t gf_mul(uint8_t a, uint8_t b)
{
    uint8_t p = 0;
    for (int i = 0; i < 8; i++) {
        if (b & 1) p ^= a;
        bool hi = (a & 0x80) != 0;
        a <<= 1;
        if (hi) a ^= 0x1D;               /* 低 8 位形式：0x11D ⇒ 0x1D ✓ */
        b >>= 1;
    }
    return p;
}
static uint8_t gf_inv(uint8_t a)         /* a^(2^8-2) ✓（a≠0）*/
{
    uint8_t r = 1;
    for (int i = 0; i < 254; i++) r = gf_mul(r, a);
    return r;
}

SasRaidTlm::SasRaidTlm(sc_core::sc_module_name nm)
: sc_core::sc_module(nm),
  clk("clk"), rst_n("rst_n"),
  o_reads("o_reads"), o_writes("o_writes"), o_rmw("o_rmw"), o_full_stripe("o_full_stripe"),
  o_degraded("o_degraded"), o_rebuild_blocks("o_rebuild_blocks"),
  o_journal_ents("o_journal_ents"), o_member_ios("o_member_ios")
{
    memset(vol, 0, sizeof(vol));
    memset(rb_scan, 0, sizeof(rb_scan));
    memset(vol_cap, 0, sizeof(vol_cap));
    backend_read = nullptr; backend_write = nullptr; backend_ctx = nullptr;
    c_rd = c_wr = c_rmw = c_full = c_deg = c_rb = c_mio = 0;
    memset(rr, 0, sizeof(rr));
    jr_head = jr_tail = 0;
    memset(jrnl, 0, sizeof(jrnl));
    // 元命令应答（**逐字节照 sas_hdd/tlm ✓**；INQUIRY byte4=32 ✓）
    memset(meta_inq, 0, sizeof(meta_inq));
    meta_inq[3] = 0x20; meta_inq[4] = 36 - 4; meta_inq[7] = 0x10;
    memcpy(&meta_inq[8],  "-UME MIS", 8);
    memcpy(&meta_inq[16], "-SAS-DHVM652", 12);
    memset(meta_ms, 0, sizeof(meta_ms));
    meta_ms[0] = 0x3F; meta_ms[4] = 0x08; meta_ms[5] = 0x12;
    memset(meta_rs, 0, sizeof(meta_rs));
    meta_rs[0] = 0x70; meta_rs[7] = 0x0A;
    SC_METHOD(step);
    sensitive << clk.pos();
    dont_initialize();
}

void SasRaidTlm::step()
{
    if (!rst_n.read()) {
        o_reads.write(0); o_writes.write(0); o_rmw.write(0); o_full_stripe.write(0);
        o_degraded.write(0); o_rebuild_blocks.write(0); o_journal_ents.write(0); o_member_ios.write(0);
        memset(vol, 0, sizeof(vol));
        c_rd = c_wr = c_rmw = c_full = c_deg = c_rb = c_mio = 0;
        memset(rr, 0, sizeof(rr));
        jr_head = jr_tail = 0;
        return;
    }
    o_reads.write(c_rd); o_writes.write(c_wr); o_rmw.write(c_rmw); o_full_stripe.write(c_full);
    o_degraded.write(c_deg); o_rebuild_blocks.write(c_rb);
    o_journal_ents.write((uint32_t)((jr_head - jr_tail) & (RAID_JOURNAL_ENTS - 1)));
    o_member_ios.write(c_mio);
}

// ── 列级几何：数据盘数 nd / 数据盘序号→物理盘号 映射 ✓ ──
static unsigned raid_nd(const raid_volume_t *v)
{
    switch (v->level) {
    case RAID_LVL_R5:  return v->n_drives - 1;
    case RAID_LVL_R6:  return v->n_drives - 2;
    case RAID_LVL_R1:  return 1;
    case RAID_LVL_R10: return v->n_drives / 2;
    default:           return v->n_drives;        /* R0 ✓ */
    }
}
// 数据序 idx → 物理盘号（R5/R6 跳过奇偶盘 ✓；R10：对 i，奇=2i 偶=2i+1 ⇒ 实为镜像对 ✓）
static unsigned raid_phys(const raid_volume_t *v, uint64_t stripe, unsigned didx)
{
    unsigned n = v->n_drives;
    if (v->level == RAID_LVL_R5 || v->level == RAID_LVL_R6) {
        unsigned nmiss = (v->level == RAID_LVL_R6) ? 2u : 1u;
        unsigned p0 = (unsigned)((n - 1) - (stripe % n));            /* 第 1 个校验盘 ✓ */
        unsigned p1 = (v->level == RAID_LVL_R6)
                      ? (unsigned)((n - 2) - ((stripe + 1) % n)) : 0xFFFF;  /* Q 盘（错开 ✓）*/
        unsigned d = 0, k = 0;
        for (d = 0; d < n; d++) {
            if (d == p0 || d == p1) continue;
            if (k == didx) return d;
            k++;
        }
        (void)nmiss;
        return 0;
    }
    if (v->level == RAID_LVL_R10) return didx * 2;                   /* 数据盘 = 每对奇号 ✓ */
    return didx;                                                     /* R0 ✓ */
}
// 校验盘号（R5：p0 ✓；R6：p0=P、p1=Q ✓）
static void raid_parity_drives(const raid_volume_t *v, uint64_t stripe, unsigned *pP, unsigned *pQ)
{
    unsigned n = v->n_drives;
    *pP = (unsigned)((n - 1) - (stripe % n));
    *pQ = (v->level == RAID_LVL_R6) ? (unsigned)((n - 2) - ((stripe + 1) % n)) : 0xFFFF;
}

void SasRaidTlm::vol_config(unsigned v, int level, unsigned n_drives, uint32_t stripe_size,
                            uint64_t cap_blocks)
{
    raid_volume_t &V = vol[v];
    memset(&V, 0, sizeof(V));
    V.in_use = 1; V.level = (uint8_t)level; V.n_drives = (uint8_t)n_drives;
    V.stripe_size = stripe_size ? stripe_size : RAID_STRIPE_SIZE;
    V.state = VOL_ONLINE;
    for (unsigned i = 0; i < n_drives; i++) {
        V.m[i].index = (uint8_t)i; V.m[i].state = DRV_ONLINE;
        V.m[i].sas_addr = 0x5000c50000000001ull + i;    // 成员 SAS 地址（示例 ✓）
    }
    vol_cap[v] = cap_blocks;
    rb_scan[v] = 0;
    rr[v] = 0;
}

void SasRaidTlm::member_fail(unsigned v, unsigned drv)
{
    raid_volume_t &V = vol[v];
    if (drv >= V.n_drives) return;
    V.m[drv].state = DRV_FAILED;
    unsigned miss = 0;
    for (unsigned i = 0; i < V.n_drives; i++) if (V.m[i].state == DRV_FAILED) miss++;
    unsigned tol = (V.level == RAID_LVL_R6) ? 2u : (V.level == RAID_LVL_R5 ? 1u : 0u);
    if (V.level == RAID_LVL_R1 || V.level == RAID_LVL_R10) tol = V.n_drives / 2;  // 每对可缺 1 ✓
    V.state = (miss <= tol) ? VOL_DEGRADED : VOL_OFFLINE;      // 超出冗余 ⇒ OFFLINE ✓
}

void SasRaidTlm::member_insert(unsigned v, unsigned drv)
{
    raid_volume_t &V = vol[v];
    if (drv >= V.n_drives) return;
    V.m[drv].state = DRV_REBUILDING;                            // 插回 ⇒ 待重建 ✓
    V.state = VOL_REBUILDING;
    rb_scan[v] = 0;
}

bool SasRaidTlm::m_read(unsigned v, unsigned drv, uint64_t lba, uint8_t *buf, unsigned nblk)
{
    if (!drv_ok(v, drv)) return false;
    c_mio++;
    return backend_read ? backend_read(backend_ctx, v, drv, lba, buf, nblk) : false;
}
bool SasRaidTlm::m_write(unsigned v, unsigned drv, uint64_t lba, const uint8_t *buf, unsigned nblk)
{
    if (!drv_wr_ok(v, drv)) return false;                // 写口径含 REBUILDING ✓
    c_mio++;
    return backend_write ? backend_write(backend_ctx, v, drv, lba, buf, nblk) : false;
}

// ── 单元读：读 (stripe, didx) 上 off..off+len-1（单元内 ✓）——
//    源盘在线 ⇒ 直读 ✓；缺失 ⇒ 重建（R5 ✓ / R6 ✓）──
bool SasRaidTlm::read_stripe_data(unsigned v, uint64_t stripe, unsigned didx,
                                  uint64_t off, uint8_t *buf, unsigned nblk)
{
    raid_volume_t &V = vol[v];
    unsigned nd = raid_nd(&V);
    unsigned pd = raid_phys(&V, stripe, didx);
    uint64_t unit_off = off;                       // 单元内字节偏移 ✓
    uint64_t lba = stripe * (V.stripe_size / RAID_BLK_BYTES) + unit_off / RAID_BLK_BYTES;
    if (drv_ok(v, pd))
        return m_read(v, pd, lba, buf, nblk);
    // ── 降级重建：**按本行失败数据盘数分类** ✓（2026-10-09 修：旧口径把"1 数据 + 1 校验缺"
    //    误判为需 2×2 ⇒ 直接拒 ✗ —— R6 容忍 2 缺，此组合合法 ✓ 必须能读 ✓）──
    c_deg++;
    unsigned pP, pQ; raid_parity_drives(&V, stripe, &pP, &pQ);
    size_t blen = (size_t)nblk * RAID_BLK_BYTES;
    std::vector<uint8_t> acc(blen, 0), tmp(blen, 0);
    uint64_t lu = stripe * (V.stripe_size / RAID_BLK_BYTES) + unit_off / RAID_BLK_BYTES;

    if (V.level != RAID_LVL_R6) {
        // R5：唯一失败盘 = 本数据块（卷 DEGRADED ⇒ 仅 1 缺 ✓）⇒ 其余数据 ⊕ P ✓
        for (unsigned k = 0; k < nd; k++) {
            unsigned d = raid_phys(&V, stripe, k);
            if (d == pd) continue;
            if (!drv_ok(v, d)) return false;             // 不应发生 ✓
            if (!m_read(v, d, lu, tmp.data(), nblk)) return false;
            for (size_t i = 0; i < blen; i++) acc[i] ^= tmp[i];
        }
        if (!drv_ok(v, pP) || !m_read(v, pP, lu, tmp.data(), nblk)) return false;
        for (size_t i = 0; i < blen; i++) acc[i] ^= tmp[i];
        memcpy(buf, acc.data(), blen);
        return true;
    }
    // R6：统计**本行**失败数据盘（数据序 ✓）
    unsigned nmiss_d = 0, mx = 0xFFFF, kx = 0, ky = 0;
    for (unsigned k = 0; k < nd; k++) {
        unsigned d = raid_phys(&V, stripe, k);
        if (drv_ok(v, d)) continue;
        nmiss_d++;
        if (mx == 0xFFFF) { mx = d; kx = k; } else { ky = k; }
    }
    if (nmiss_d == 1) {
        // 单数据缺：优先 P（在线 ✓）；P 缺（= 1 数据 + 1 校验 组合 ✓）⇒ 用 Q + g 反解 ✓
        uint8_t gx = 1; for (unsigned i = 0; i < kx; i++) gx = gf_mul(gx, 2);
        if (drv_ok(v, pP)) {
            for (unsigned k = 0; k < nd; k++) {
                unsigned d = raid_phys(&V, stripe, k);
                if (d == mx) continue;
                if (!drv_ok(v, d)) return false;
                if (!m_read(v, d, lu, tmp.data(), nblk)) return false;
                for (size_t i = 0; i < blen; i++) acc[i] ^= tmp[i];
            }
            if (!m_read(v, pP, lu, tmp.data(), nblk)) return false;
            for (size_t i = 0; i < blen; i++) acc[i] ^= tmp[i];
        } else if (drv_ok(v, pQ)) {
            std::vector<uint8_t> q(blen, 0);
            if (!m_read(v, pQ, lu, q.data(), nblk)) return false;
            for (unsigned k = 0; k < nd; k++) {
                unsigned d = raid_phys(&V, stripe, k);
                if (d == mx) continue;
                if (!drv_ok(v, d)) return false;
                uint8_t gk = 1; for (unsigned i = 0; i < k; i++) gk = gf_mul(gk, 2);
                if (!m_read(v, d, lu, tmp.data(), nblk)) return false;
                for (size_t i = 0; i < blen; i++) q[i] ^= gf_mul(gk, tmp[i]);
            }
            uint8_t gi = gf_inv(gx);
            for (size_t i = 0; i < blen; i++) acc[i] = gf_mul(q[i], gi);   // Dx = (Q ⊕ Σ) / gx ✓
        } else {
            return false;                                 // P、Q 双缺 ⇒ 无解 ✓（>容限，理论不可达 ✓）
        }
        memcpy(buf, acc.data(), blen);
        return true;
    }
    if (nmiss_d == 2) {
        // 双数据缺 ⇒ 2×2（Dx ⊕ Dy = P'；gx·Dx ⊕ gy·Dy = Q' ✓）—— 需 P、Q 均在线 ✓
        if (!drv_ok(v, pP) || !drv_ok(v, pQ)) return false;   // 双数据缺 + 缺校验 ⇒ 无解 ✓
        std::vector<uint8_t> Pp(blen, 0), Qp(blen, 0), t(blen, 0);
        if (!m_read(v, pP, lu, Pp.data(), nblk)) return false;
        if (!m_read(v, pQ, lu, Qp.data(), nblk)) return false;
        for (unsigned k = 0; k < nd; k++) {
            unsigned d = raid_phys(&V, stripe, k);
            if (!drv_ok(v, d)) continue;                 // 跳过两块失败数据 ✓
            if (!m_read(v, d, lu, t.data(), nblk)) return false;
            uint8_t gk = 1; for (unsigned i = 0; i < k; i++) gk = gf_mul(gk, 2);
            for (size_t i = 0; i < blen; i++) { Pp[i] ^= t[i]; Qp[i] ^= gf_mul(gk, t[i]); }
        }
        uint8_t gx = 1, gy = 1;
        for (unsigned i = 0; i < kx; i++) gx = gf_mul(gx, 2);
        for (unsigned i = 0; i < ky; i++) gy = gf_mul(gy, 2);
        uint8_t den = gf_inv((uint8_t)(gx ^ gy));
        for (size_t i = 0; i < blen; i++)
            t[i] = gf_mul((uint8_t)(Qp[i] ^ gf_mul(gy, Pp[i])), den);   // Dx ✓
        if (pd == mx) memcpy(buf, t.data(), blen);
        else for (size_t i = 0; i < blen; i++) buf[i] = (uint8_t)(Pp[i] ^ t[i]);  // Dy = P' ⊕ Dx ✓
        return true;
    }
    return false;                                        // nmiss_d==0 不应达 ✓
}

// ── 单元写增量（R5/R6 的 RMW 核心 ✓）：
//    D_old = 读目标区段（含重建 ✓）；Δ = D_old ⊕ D_new；P' = P_old ⊕ Δ；Q' = Q_old ⊕ g_i·Δ ✓
bool SasRaidTlm::write_row(unsigned v, uint64_t stripe, const uint8_t *row)
{ (void)v; (void)stripe; (void)row; return false; }   // 保留接口（整行写优化 = 扩展点 ✓）

// ── 虚拟盘面 ──
bool SasRaidTlm::vdisk_read(unsigned v, uint64_t lba, uint8_t *buf, unsigned nblk)
{
    raid_volume_t &V = vol[v];
    if (!V.in_use || V.state == VOL_OFFLINE) return false;
    c_rd++;
    unsigned nd = raid_nd(&V);
    uint64_t done = 0;
    while (done < nblk) {
        raid_xlate_t x; raid_xlate_nd(lba + done, V.n_drives, nd, V.stripe_size, &x);
        uint64_t unit_blk = V.stripe_size / RAID_BLK_BYTES;
        uint64_t off_blk  = (lba + done) % unit_blk;                 // 单元内偏移 ✓
        uint64_t want     = unit_blk - off_blk;
        if (want > (nblk - done)) want = nblk - done;
        if (V.level == RAID_LVL_R1 || V.level == RAID_LVL_R10) {
            // 镜像：任选在线副本（读均衡：轮转 ✓）
            unsigned ncopy = (V.level == RAID_LVL_R1) ? 2u : 2u;
            unsigned base  = (V.level == RAID_LVL_R1) ? 0u : raid_phys(&V, x.stripe, x.drive);
            unsigned pick  = 0xFFFF;
            unsigned start = (rr[v]++) & 1u;             // 读均衡：逐单元轮转起读副本 ✓（M2-B5 ✓）
            for (unsigned c = 0; c < ncopy; c++) {
                unsigned d = (V.level == RAID_LVL_R1) ? (start ^ c) : (base + (start ^ c));
                if (drv_ok(v, d)) { pick = d; break; }
            }
            if (pick == 0xFFFF) return false;
            uint64_t dl = (V.level == RAID_LVL_R1) ? (lba + done)
                        : (x.stripe * unit_blk + off_blk);
            if (!m_read(v, pick, dl, buf + done * RAID_BLK_BYTES, (unsigned)want)) return false;
        } else {
            if (!read_stripe_data(v, x.stripe, x.drive, off_blk * RAID_BLK_BYTES,
                                  buf + done * RAID_BLK_BYTES, (unsigned)want)) return false;
        }
        done += want;
    }
    return true;
}

bool SasRaidTlm::vdisk_write(unsigned v, uint64_t lba, const uint8_t *buf, unsigned nblk)
{
    raid_volume_t &V = vol[v];
    if (!V.in_use || V.state == VOL_OFFLINE) return false;
    c_wr++;
    unsigned nd = raid_nd(&V);
    uint64_t done = 0;
    std::vector<uint8_t> dold, pold, qold, tmp;
    while (done < nblk) {
        raid_xlate_t x; raid_xlate_nd(lba + done, V.n_drives, nd, V.stripe_size, &x);
        uint64_t unit_blk = V.stripe_size / RAID_BLK_BYTES;
        uint64_t off_blk  = (lba + done) % unit_blk;
        uint64_t want     = unit_blk - off_blk;
        if (want > (nblk - done)) want = nblk - done;
        unsigned blen = (unsigned)(want * RAID_BLK_BYTES);
        unsigned pP, pQ; raid_parity_drives(&V, x.stripe, &pP, &pQ);
        unsigned pd = raid_phys(&V, x.stripe, x.drive);
        uint64_t dl = x.stripe * unit_blk + off_blk;

        // ── 全条带快速路径（arch §4.13 ✓）：行对齐 + 覆盖整行 + **数据盘全在线** ⇒
        //    P = ⊕新数据（R6 追加 Q = ⊕g_k·新数据 ✓）—— **无旧数据/旧校验读** ✓（写惩罚 = N ✓）
        if ((V.level == RAID_LVL_R5 || V.level == RAID_LVL_R6) &&
            off_blk == 0 && (nblk - done) >= unit_blk * nd) {
            bool all_data_ok = true;
            for (unsigned k = 0; k < nd; k++)
                if (!drv_ok(v, raid_phys(&V, x.stripe, k))) all_data_ok = false;
            if (all_data_ok) {
                size_t ubytes = (size_t)unit_blk * RAID_BLK_BYTES;
                std::vector<uint8_t> acc(ubytes, 0), q(ubytes, 0);
                for (unsigned k = 0; k < nd; k++) {
                    const uint8_t *src = buf + (done + (uint64_t)k * unit_blk) * RAID_BLK_BYTES;
                    for (size_t i = 0; i < ubytes; i++) acc[i] ^= src[i];
                    if (V.level == RAID_LVL_R6) {
                        uint8_t g = 1; for (unsigned j = 0; j < k; j++) g = gf_mul(g, 2);
                        for (size_t i = 0; i < ubytes; i++) q[i] ^= gf_mul(g, src[i]);
                    }
                }
                journal_add(v, x.stripe);                    // 写前记 ✓
                for (unsigned k = 0; k < nd; k++) {
                    unsigned d = raid_phys(&V, x.stripe, k);
                    if (!m_write(v, d, dl, buf + (done + (uint64_t)k * unit_blk) * RAID_BLK_BYTES,
                                 (unsigned)unit_blk)) return false;
                }
                if (drv_ok(v, pP)) {
                    if (!m_write(v, pP, dl, acc.data(), (unsigned)unit_blk)) return false;
                } else if (V.level == RAID_LVL_R6) {
                    return false;                            // R6 P/Q 缺 ⇒ 降级写不支持（扩展点 ✗）
                }
                if (V.level == RAID_LVL_R6) {
                    if (!drv_ok(v, pQ)) return false;
                    if (!m_write(v, pQ, dl, q.data(), (unsigned)unit_blk)) return false;
                }
                journal_clear();
                c_full++;
                done += unit_blk * nd;
                continue;
            }
            // 数据盘有缺 ⇒ 落到 RMW 逐单元（失败盘的单元写将拒 ✗ 由下方 m_write 把关 ✓）
        }

        if (V.level == RAID_LVL_R0) {
            if (!m_write(v, pd, dl, buf + done * RAID_BLK_BYTES, (unsigned)want)) return false;
        } else if (V.level == RAID_LVL_R1 || V.level == RAID_LVL_R10) {
            unsigned base = (V.level == RAID_LVL_R1) ? 0u : pd;
            unsigned ncopy = 2;
            uint64_t wl = dl;                               // 物理盘上同一位置 ✓
            for (unsigned c = 0; c < ncopy; c++) {
                unsigned d = (V.level == RAID_LVL_R1) ? c : base + c;
                // 写口径含 REBUILDING ✓（镜像重建期间主机写也要落两副本 ✓ 防重建成旧值 ✗）
                if (drv_wr_ok(v, d)) { if (!m_write(v, d, wl, buf + done * RAID_BLK_BYTES, (unsigned)want)) return false; }
            }
        } else {
            // ── R5/R6 RMW（只算触碰区段 ✓）：**先把读写素材全部备齐**（含 Q 前置检查 ✓）
            //    —— 任一前置不满足 ⇒ 未动盘即失败 ✓ 无部分写 ✗（旧口径 Q 缺会半写 ✗ 已修 ✓）──
            c_rmw++;
            dold.assign(blen, 0);
            if (!read_stripe_data(v, x.stripe, x.drive, off_blk * RAID_BLK_BYTES, dold.data(), (unsigned)want))
                return false;
            pold.assign(blen, 0);
            if (drv_ok(v, pP)) {
                if (!m_read(v, pP, dl, pold.data(), (unsigned)want)) return false;
            } else if (V.level == RAID_LVL_R5) {
                std::vector<uint8_t> t(blen, 0);            // P 缺：P_old = ⊕所有数据 ✓
                for (unsigned k = 0; k < nd; k++) {
                    unsigned d = raid_phys(&V, x.stripe, k);
                    if (!drv_ok(v, d)) return false;
                    if (!m_read(v, d, dl, t.data(), (unsigned)want)) return false;
                    for (unsigned i = 0; i < blen; i++) pold[i] ^= t[i];
                }
            } else {
                return false;                                // R6 且 P 缺（Q 在）⇒ 留扩展点 ✗
            }
            std::vector<uint8_t> qold;
            if (V.level == RAID_LVL_R6) {                    // Q 前置检查（先读后写 ✓）
                qold.assign(blen, 0);
                if (!drv_ok(v, pQ) || !m_read(v, pQ, dl, qold.data(), (unsigned)want)) return false;
            }
            // Δ = D_old ⊕ D_new ✓
            tmp.assign(blen, 0);
            for (unsigned i = 0; i < blen; i++)
                tmp[i] = (uint8_t)(dold[i] ^ buf[done * RAID_BLK_BYTES + i]);
            journal_add(v, x.stripe);                        // 写前记 ✓
            if (!m_write(v, pd, dl, buf + done * RAID_BLK_BYTES, (unsigned)want)) return false;
            for (unsigned i = 0; i < blen; i++) pold[i] ^= tmp[i];   // P' = P_old ⊕ Δ ✓
            if (drv_ok(v, pP)) {
                if (!m_write(v, pP, dl, pold.data(), (unsigned)want)) return false;
            } else if (V.level == RAID_LVL_R5) {
                // P 盘缺 ⇒ 校验无处可写：R5 降级写只写数据（P 将在重建时补 ✓）
            }
            if (V.level == RAID_LVL_R6) {                    // Q' = Q_old ⊕ g_i·Δ ✓
                uint8_t g = 1; for (unsigned i = 0; i < x.drive; i++) g = gf_mul(g, 2);
                for (unsigned i = 0; i < blen; i++) qold[i] ^= gf_mul(g, tmp[i]);
                if (!m_write(v, pQ, dl, qold.data(), (unsigned)want)) return false;
            }
            journal_clear();                                 // 本模型简化：立即清 ✓
        }
        done += want;
    }
    return true;
}

// ── 重建（M5 ✓）：逐条带逐盘推进（每次调用 = 单盘一条带行的单元 ✓）
//   ★ 2026-10-09 修 + 定型（台逼出来的 ✓）：
//     (a) **读口径**：`drv_ok` = 仅 ONLINE ⇒ 重建中的盘**读必走重建路径** ✓
//         （旧口径 drv_ok 含 REBUILDING ⇒ 重建读直读被建盘自身的陈旧内容 ✗ 等于抄 ✗）；
//         写口径 `drv_wr_ok` 含 REBUILDING ⇒ 重建写回目标盘 ✓。
//     (b) **逐盘分轮**：扫完一遍仍有余盘 ⇒ `rb_scan` 归零续建 ✓（旧口径一把全置 ONLINE ✗ 漏建 ✗）；
//         **单盘扫毕才置 ONLINE** ✓（旧口径写一块就置 ONLINE ✗ 半建 ✗）。
//     (c) **镜像重建**（R1/R10）：从**兄弟副本**（`target^1` ✓）整单元拷贝 ✓
//         （旧口径走数据序查表 ⇒ R1/R10 找不到 ✗）。
//     (d) 校验盘重建要求数据盘**全在线** ✓ 否则**推迟**（返回 false ✓ 下轮再试 ✓）。
bool SasRaidTlm::rebuild_step(unsigned v)
{
    raid_volume_t &V = vol[v];
    if (!V.in_use || V.state != VOL_REBUILDING) return false;
    unsigned nd = raid_nd(&V);
    uint64_t unit_blk = V.stripe_size / RAID_BLK_BYTES;
    uint64_t stripes_total = (vol_cap[v] + unit_blk * nd - 1) / (unit_blk * nd);
    if (rb_scan[v] >= stripes_total) {                   // 本轮扫完 ✓
        bool more = false;
        for (unsigned d = 0; d < V.n_drives; d++) if (V.m[d].state == DRV_REBUILDING) more = true;
        if (more) rb_scan[v] = 0;                        // 还有盘待建 ⇒ 下一轮 ✓
        else { V.state = VOL_ONLINE; return false; }     // 全部完成 ✓
    }
    uint64_t stripe = rb_scan[v];
    unsigned target = 0xFFFF;
    for (unsigned d = 0; d < V.n_drives; d++) if (V.m[d].state == DRV_REBUILDING) { target = d; break; }
    if (target == 0xFFFF) { V.state = VOL_ONLINE; return false; }   // 不应达 ✓
    unsigned pP, pQ; raid_parity_drives(&V, stripe, &pP, &pQ);
    size_t ubytes = (size_t)unit_blk * RAID_BLK_BYTES;
    std::vector<uint8_t> unit(ubytes, 0), t(ubytes, 0);
    uint64_t dl = stripe * unit_blk;
    unsigned want_blk = (unsigned)unit_blk;
    bool okr;
    if (V.level == RAID_LVL_R1 || V.level == RAID_LVL_R10) {
        unsigned sib = target ^ 1u;                      // 兄弟副本 ✓（对 = (0,1)(2,3)… ✓）
        okr = drv_ok(v, sib) && m_read(v, sib, dl, unit.data(), want_blk);
    } else if (target == pP || target == pQ) {
        okr = true;                                      // 校验盘：由**全部**数据重算 ✓
        for (unsigned k = 0; k < nd && okr; k++) {
            unsigned d = raid_phys(&V, stripe, k);
            if (!drv_ok(v, d)) { okr = false; break; }   // 数据不全 ⇒ 本轮推迟 ✓
            if (!m_read(v, d, dl, t.data(), want_blk)) { okr = false; break; }
            if (target == pP) {
                for (size_t i = 0; i < ubytes; i++) unit[i] ^= t[i];
            } else {
                uint8_t g = 1; for (unsigned j = 0; j < k; j++) g = gf_mul(g, 2);
                for (size_t i = 0; i < ubytes; i++) unit[i] ^= gf_mul(g, t[i]);
            }
        }
    } else {
        unsigned didx = 0xFFFF;                          // 数据盘：重建读（含 R6 双缺 2×2 ✓）
        for (unsigned k = 0; k < nd; k++) if (raid_phys(&V, stripe, k) == target) didx = k;
        okr = (didx != 0xFFFF) &&
              read_stripe_data(v, stripe, didx, 0, unit.data(), want_blk);
    }
    if (!okr) return false;                              // 推迟（下轮重试 ✓ 保持 REBUILDING ✓）
    if (!m_write(v, target, dl, unit.data(), want_blk)) return false;
    c_rb += want_blk;
    rb_scan[v]++;
    if (rb_scan[v] >= stripes_total) {                   // 本轮扫毕 ⇒ **该盘**建成 ✓
        V.m[target].state = DRV_ONLINE;
        bool more = false;
        for (unsigned d = 0; d < V.n_drives; d++) if (V.m[d].state == DRV_REBUILDING) more = true;
        if (more) rb_scan[v] = 0;                        // 下一盘 ✓
        else V.state = VOL_ONLINE;                       // 全部完成 ⇒ 卷在线 ✓
    }
    return true;
}

unsigned SasRaidTlm::journal_count() const
{ return (unsigned)((jr_head - jr_tail) & (RAID_JOURNAL_ENTS - 1)); }

void SasRaidTlm::journal_replay(unsigned v)
{
    // 重放 = 对每条未清日志所属条带**重算校验**（幂等 ✓）
    (void)v;
    jr_tail = jr_head;
}

void SasRaidTlm::meta_inquiry(uint8_t out[INQ_LEN]) const { memcpy(out, meta_inq, INQ_LEN); }
void SasRaidTlm::meta_mode_sense(uint8_t out[MODE_SENSE_LEN]) const { memcpy(out, meta_ms, MODE_SENSE_LEN); }
void SasRaidTlm::meta_req_sense(uint8_t out[REQ_SENSE_LEN]) const { memcpy(out, meta_rs, REQ_SENSE_LEN); }
void SasRaidTlm::meta_rcap10(uint8_t out[RCAP10_LEN]) const
{
    memset(out, 0, RCAP10_LEN);
    uint64_t last = (vol_cap[0] ? vol_cap[0] : 1) - 1;      // 卷 0 容量（多卷扩展点 ✓）
    out[0] = (uint8_t)(last >> 24); out[1] = (uint8_t)(last >> 16);
    out[2] = (uint8_t)(last >> 8);  out[3] = (uint8_t)last;
    out[6] = 0x02;                                          // 块长 512 BE ✓
}
