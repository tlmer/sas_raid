//============================================================================
// tb_raid_r6.cpp — M4 判据台：RAID 6 双校验（P + Q）  [2026-10-09 建]
//----------------------------------------------------------------------------
//   配置：6 盘 / 64KB 条带 / 行 = 4×128 = 512 块 ✓（nd=4 ✓；stripe0：P=5/Q=3 ✓ 错开 ✓）
//   判据（**预登记** ✓；出处 = `内部内部开发计划` §3 M4 + arch §4.13 Dual Parity ✓）：
//   D1 **全条带写**（stripe0 512 块）⇒ ① 全盘逐字节 == 独立参考 ✓ ② 后端零读 ✓
//      ③ 每盘恰 1 写 ✓ ④ `c_full==1` ✓ ⑤ **独立 GF 恒等式**：P == ⊕D ✓、
//      Q == ⊕ g_k·D_k ✓ —— TB 用 **exp/log 表乘法**（与模型位算法**不同算法** ✓ 防同错 ✗）
//   D2 **RMW 部分写**（100..119）⇒ 对拍 ✓ + 写惩罚签名 3 读 3 写（D_old/P/Q ✓）+ c_rmw==1 ✓
//   D3 **跨单元跨行写**（500..539）⇒ 对拍 ✓ + c_rmw 增量 == 2 ✓
//   D4 **行1 全条带**（512..1023）⇒ 对拍 ✓ c_full==2 ✓ + **两行** P/Q 独立恒等式 ✓
//   D5 **单缺读**（拔盘 1）⇒ 读 0..1023 == 期望 ✓（P 路径 ✓ c_deg 增长 ✓）
//   D6 **双缺读**（再拔盘 2）⇒ 读 0..1023 == 期望 ✓——且**两行走不同路径** ✓：
//      stripe0：双数据缺 ⇒ **2×2 GF 解** ✓；stripe1：1 数据 + Q 缺 ⇒ **新修路径**（P 重建 ✓）
//   D7 **独立卷 vol1 双缺**（拔 {1,3}）⇒ 读 == 期望 ✓（stripe0: 1 数据+Q ⇒ P 路 ✓；
//      stripe1: 双数据缺 ⇒ 2×2 ✓ —— 与 D6 **反向**覆盖两条路径 ✓）
//   D8 **负控**：vol1 再拔盘 5 ⇒ 3 缺超冗余 ⇒ OFFLINE ✓ 读写皆拒 ✓
//============================================================================
#include <cstdio>
#include <cstring>
#include <vector>
#include "raid_tb_common.h"

// ── 独立 GF(2⁸)：exp/log 表（poly 0x11D ✓ 生成元 2 ✓）—— 与模型位算法**不同算法** ✓ ──
struct GFTable {
    uint8_t ex[512], lg[256];
    GFTable() {
        unsigned x = 1;
        for (unsigned i = 0; i < 255; i++) {
            ex[i] = (uint8_t)x; lg[x] = (uint8_t)i;
            x <<= 1; if (x & 0x100) x ^= 0x11D;
        }
        for (unsigned i = 255; i < 512; i++) ex[i] = ex[i - 255];
        lg[0] = 0;
    }
    uint8_t mul(uint8_t a, uint8_t b) const {
        if (!a || !b) return 0;
        return ex[(unsigned)lg[a] + (unsigned)lg[b]];
    }
    uint8_t pow2(unsigned k) const { return ex[k % 255]; }
};

// ── 独立参考模型（逐块 spec 公式 + 独立旋转 + 增量 P/Q ✓）──
struct RefR6 {
    unsigned n, nd; uint64_t ss; size_t cap;
    GFTable gf;
    std::vector<std::vector<uint8_t>> m;
    RefR6(unsigned n_, unsigned nd_, uint64_t ss_, size_t cap_)
        : n(n_), nd(nd_), ss(ss_), cap(cap_), m(n_, std::vector<uint8_t>(cap_, 0)) {}
    void par(uint64_t stripe, unsigned *pP, unsigned *pQ) const {
        *pP = (unsigned)((n - 1) - (stripe % n));
        *pQ = (unsigned)(((n - 2) + n - ((stripe + 1) % n)) % n);   // 回绕 ✓（与模型同修 ✓）
    }
    unsigned phys(unsigned k, uint64_t stripe) const {
        unsigned pP, pQ; par(stripe, &pP, &pQ); unsigned cnt = 0;
        for (unsigned d = 0; d < n; d++) {
            if (d == pP || d == pQ) continue;
            if (cnt == k) return d;
            cnt++;
        }
        return 0;
    }
    void put_sub(uint64_t lba, const uint8_t *buf, unsigned nblk) {
        for (unsigned i = 0; i < nblk; i++) {
            uint64_t b = lba + i, bytes = b * 512, row = (uint64_t)ss * nd;
            uint64_t stripe = bytes / row, off = bytes % row;
            unsigned k = (unsigned)(off / ss);
            uint64_t lba_m = stripe * (ss / 512) + (off % ss) / 512;
            unsigned pP, pQ; par(stripe, &pP, &pQ);
            unsigned d = phys(k, stripe);
            uint8_t *dst = &m[d][(size_t)lba_m * 512];
            uint8_t *p  = &m[pP][(size_t)lba_m * 512];
            uint8_t *q  = &m[pQ][(size_t)lba_m * 512];
            const uint8_t *src = buf + (size_t)i * 512;
            for (unsigned j = 0; j < 512; j++) {
                uint8_t delta = (uint8_t)(dst[j] ^ src[j]);
                p[j] ^= delta;
                q[j] ^= gf.mul(gf.pow2(k), delta);       // Q' = Q ⊕ g_k·Δ ✓
                dst[j] = src[j];
            }
        }
    }
    bool eq(const RaidMemBackend &bk, unsigned v) const {
        for (unsigned d = 0; d < n; d++)
            if (std::memcmp(bk.mem[v][d].data(), m[d].data(), cap) != 0) return false;
        return true;
    }
    void diff(const RaidMemBackend &bk, unsigned v) const {
        for (unsigned d = 0; d < n; d++) {
            if (std::memcmp(bk.mem[v][d].data(), m[d].data(), cap) == 0) continue;
            for (size_t i = 0; i < cap; i++)
                if (bk.mem[v][d][i] != m[d][i]) { printf("      [diff] drv%u off %zu\n", d, i); break; }
        }
    }
};

// ── 独立 P/Q 恒等式：由成员数据盘**现读字节**重算 P/Q 并比对成员校验盘 ✓ ──
static bool pq_identity(const RaidMemBackend &bk, unsigned v, uint64_t stripe,
                        unsigned n, unsigned nd, uint64_t ss, const GFTable &gf)
{
    unsigned pP = (unsigned)((n - 1) - (stripe % n));
    unsigned pQ = (unsigned)(((n - 2) + n - ((stripe + 1) % n)) % n);   // 回绕 ✓
    std::vector<uint8_t> P(ss, 0), Q(ss, 0);
    unsigned k = 0;
    for (unsigned d = 0; d < n; d++) {
        if (d == pP || d == pQ) continue;
        const uint8_t *u = &bk.mem[v][d][stripe * ss];
        for (size_t i = 0; i < ss; i++) { P[i] ^= u[i]; Q[i] ^= gf.mul(gf.pow2(k), u[i]); }
        k++;
    }
    return std::memcmp(&bk.mem[v][pP][stripe * ss], P.data(), ss) == 0 &&
           std::memcmp(&bk.mem[v][pQ][stripe * ss], Q.data(), ss) == 0;
}

int sc_main(int argc, char **argv)
{
    (void)argc; (void)argv;
    RaidTbScore sc;
    printf("=== tb_raid_r6：RAID 6 双校验（2×2 GF 恢复 ✓）对独立参考（M4 ✓）===\n");

    const size_t CAP = 8u * 1024u * 1024u;
    RaidMemBackend bk(CAP);
    sc_core::sc_signal<bool> clk_s, rst_s;
    sc_core::sc_signal<sc_uint<32>> s_rd, s_wr, s_rmw, s_full, s_deg, s_rb, s_jr, s_mio;
    SasRaidTlm raid("raid");
    raid.clk(clk_s); raid.rst_n(rst_s);
    raid.o_reads(s_rd); raid.o_writes(s_wr); raid.o_rmw(s_rmw); raid.o_full_stripe(s_full);
    raid.o_degraded(s_deg); raid.o_rebuild_blocks(s_rb); raid.o_journal_ents(s_jr);
    raid.o_member_ios(s_mio);
    rst_s.write(true); clk_s.write(false);
    bk.bind(raid);

    const uint32_t SS = 65536;
    const unsigned ROW_BLK = 512;                                 // 4 数据盘 × 128 块 ✓
    raid.vol_config(0, RAID_LVL_R6, 6, SS, 4096);
    raid.vol_config(1, RAID_LVL_R6, 6, SS, 4096);
    RefR6 ref(6, 4, SS, CAP);                                     // vol0 参考 ✓
    GFTable gf;

    std::vector<uint8_t> vexp((size_t)4096 * 512, 0);   // D9 用到 stripe4/5 ⇒ 扩到 4096 块 ✓
    auto wr = [&](unsigned v, uint64_t lba, const std::vector<uint8_t> &b) {
        return raid.vdisk_write(v, lba, b.data(), (unsigned)(b.size() / 512));
    };
    auto note = [&](uint64_t lba, const std::vector<uint8_t> &b) {
        std::memcpy(&vexp[(size_t)lba * 512], b.data(), b.size());
    };
    auto rd0_eq = [&](unsigned v, const std::vector<uint8_t> &ex) {
        std::vector<uint8_t> r(ex.size(), 0);
        bool ok = raid.vdisk_read(v, 0, r.data(), (unsigned)(ex.size() / 512));
        return ok && std::memcmp(r.data(), ex.data(), ex.size()) == 0;
    };
    // ── D1 全条带写 stripe0 + 独立 GF 恒等式 ──
    {
        std::vector<uint8_t> w1((size_t)ROW_BLK * 512); pat_fill(w1.data(), w1.size(), 0xD1);
        bk.reset_cnt();
        bool okw = wr(0, 0, w1); note(0, w1); ref.put_sub(0, w1.data(), ROW_BLK);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        bool no_rd = true; for (unsigned d = 0; d < 6; d++) if (bk.rd_cnt[0][d]) no_rd = false;
        bool wr1x  = true; for (unsigned d = 0; d < 6; d++) if (bk.wr_cnt[0][d] != 1) wr1x = false;
        sc.chk(okw && okm && no_rd && wr1x && raid.cnt_full() == 1 && pq_identity(bk, 0, 0, 6, 4, SS, gf),
               "D1 全条带写：对拍 ✓ 零读 ✓ 每盘1写 ✓ c_full=1 ✓ 独立 GF 恒等式 P/Q ✓");
    }
    // ── D2 RMW 部分写 + 写惩罚 ──
    {
        std::vector<uint8_t> w2((size_t)20 * 512); pat_fill(w2.data(), w2.size(), 0xD2);
        bk.reset_cnt();
        bool okw = wr(0, 100, w2); note(100, w2); ref.put_sub(100, w2.data(), 20);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        bool pen = bk.rd_cnt[0][0] == 1 && bk.rd_cnt[0][5] == 1 && bk.rd_cnt[0][3] == 1 &&
                   bk.wr_cnt[0][0] == 1 && bk.wr_cnt[0][5] == 1 && bk.wr_cnt[0][3] == 1;
        for (unsigned d = 0; d < 6; d++)
            if (d != 0 && d != 3 && d != 5) pen = pen && !bk.rd_cnt[0][d] && !bk.wr_cnt[0][d];
        sc.chk(okw && okm && pen && raid.cnt_rmw() == 1,
               "D2 RMW：对拍 ✓ 写惩罚 3 读 3 写（D_old/P/Q ✓）c_rmw=1 ✓");
    }
    // ── D3 跨单元跨行（500..539：stripe0 尾 12 块 + stripe1 头 28 块）──
    {
        std::vector<uint8_t> w3((size_t)40 * 512); pat_fill(w3.data(), w3.size(), 0xD3);
        uint32_t r0 = raid.cnt_rmw();
        bool okw = wr(0, 500, w3); note(500, w3); ref.put_sub(500, w3.data(), 40);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        sc.chk(okw && okm && (raid.cnt_rmw() - r0 == 2), "D3 跨单元跨行：对拍 ✓ c_rmw 增量=2 ✓");
    }
    // ── D4 行1 全条带 + 两行独立恒等式 ──
    {
        std::vector<uint8_t> w4((size_t)ROW_BLK * 512); pat_fill(w4.data(), w4.size(), 0xD4);
        bool okw = wr(0, 512, w4); note(512, w4); ref.put_sub(512, w4.data(), ROW_BLK);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        sc.chk(okw && okm && raid.cnt_full() == 2 && pq_identity(bk, 0, 0, 6, 4, SS, gf) &&
               pq_identity(bk, 0, 1, 6, 4, SS, gf),
               "D4 行1 全条带：对拍 ✓ c_full=2 ✓ 两行 P/Q 独立恒等式 ✓（P/Q 位 5/3→4/2 ✓）");
    }
    // ── D9 R6 校验盘**回绕旋转**存储级金标（stripe4/5 ✓ 手排 ✓）──
    {
        std::vector<uint8_t> w9((size_t)1024 * 512); pat_fill(w9.data(), w9.size(), 0xD9);
        bool okw = wr(0, 2048, w9); note(2048, w9);
        // 手排金标（n=6 ✓ 修回绕后 ✓）：stripe4 ⇒ P@盘1 / Q@盘5，数据序 [0,2,3,4] ✓
        //                              stripe5 ⇒ P@盘0 / Q@盘4，数据序 [1,2,3,5] ✓
        const uint8_t *s4 = &vexp[(size_t)2048 * 512];   // stripe4 行首 ✓
        const uint8_t *s5 = &vexp[(size_t)(2048 + 512) * 512];
        std::vector<uint8_t> P4((size_t)128 * 512, 0), P5((size_t)128 * 512, 0);
        std::vector<uint8_t> Q4((size_t)128 * 512, 0), Q5((size_t)128 * 512, 0);
        for (unsigned k = 0; k < 4; k++) {
            const uint8_t *u4 = s4 + (size_t)k * 128 * 512;
            const uint8_t *u5 = s5 + (size_t)k * 128 * 512;
            uint8_t g = 1; for (unsigned j = 0; j < k; j++) g = gf.mul(g, 2);
            for (size_t i = 0; i < P4.size(); i++) {
                P4[i] ^= u4[i]; Q4[i] ^= gf.mul(g, u4[i]);
                P5[i] ^= u5[i]; Q5[i] ^= gf.mul(g, u5[i]);
            }
        }
        // 直查（每盘 128KB 单元 = 虚拟片 ✓）：stripe4 ⇒ 盘1 = P4 / 盘5 = Q4 ✓
        auto chk_drv = [&](unsigned d, uint64_t mofs, const uint8_t *ex) {
            return std::memcmp(&bk.mem[0][d][(size_t)mofs], ex, (size_t)128 * 512) == 0;
        };
        bool rot4 = chk_drv(0, 4 * 65536, &s4[(size_t)0 * 128 * 512]) &&
                    chk_drv(2, 4 * 65536, &s4[(size_t)1 * 128 * 512]) &&
                    chk_drv(3, 4 * 65536, &s4[(size_t)2 * 128 * 512]) &&
                    chk_drv(4, 4 * 65536, &s4[(size_t)3 * 128 * 512]) &&
                    chk_drv(1, 4 * 65536, P4.data()) && chk_drv(5, 4 * 65536, Q4.data());
        bool rot5 = chk_drv(1, 5 * 65536, &s5[(size_t)0 * 128 * 512]) &&
                    chk_drv(2, 5 * 65536, &s5[(size_t)1 * 128 * 512]) &&
                    chk_drv(3, 5 * 65536, &s5[(size_t)2 * 128 * 512]) &&
                    chk_drv(5, 5 * 65536, &s5[(size_t)3 * 128 * 512]) &&
                    chk_drv(0, 5 * 65536, P5.data()) && chk_drv(4, 5 * 65536, Q5.data());
        sc.chk(okw && rot4 && rot5 && rd0_eq(0, vexp),
               "D9 回绕旋转（stripe4 ⇒ P@1/Q@5 ✓ stripe5 ⇒ P@0/Q@4 ✓ 存储级 ✓ 手排金标 ✓）");
    }

    // ── D5 单缺读（拔盘 1）──
    {
        uint32_t d0 = raid.cnt_deg();
        raid.member_fail(0, 1);
        sc.chk(raid.vol[0].state == VOL_DEGRADED && rd0_eq(0, vexp) && (raid.cnt_deg() > d0),
               "D5 单缺读：DEGRADED ✓ 读 0..1023 == 期望 ✓（P 路径 ✓）");
    }
    // ── D6 双缺读（拔 1+2：stripe0 双数据缺⇒2×2 ✓；stripe1 单数据+Q 缺⇒P 路 ✓）──
    {
        raid.member_fail(0, 2);
        sc.chk(raid.vol[0].state == VOL_DEGRADED && rd0_eq(0, vexp),
               "D6 双缺读：DEGRADED ✓ 读 0..1023 == 期望 ✓（stripe0 2×2 ✓ / stripe1 1数据+Q缺 ✓）");
    }
    // ── D7 独立卷 vol1：拔 {1,3}（stripe0 1数据+Q ⇒ P 路 ✓；stripe1 双数据缺 ⇒ 2×2 ✓）──
    {
        std::vector<uint8_t> a((size_t)ROW_BLK * 512); pat_fill(a.data(), a.size(), 0xD7);
        std::vector<uint8_t> b((size_t)ROW_BLK * 512); pat_fill(b.data(), b.size(), 0xD8);
        bool okw = wr(1, 0, a) && wr(1, 512, b);
        std::vector<uint8_t> ex((size_t)1024 * 512, 0);
        std::memcpy(&ex[0], a.data(), a.size());
        std::memcpy(&ex[(size_t)512 * 512], b.data(), b.size());
        raid.member_fail(1, 1);
        raid.member_fail(1, 3);
        sc.chk(okw && raid.vol[1].state == VOL_DEGRADED && rd0_eq(1, ex),
               "D7 vol1 双缺 {1,3}：读 == 期望 ✓（stripe0 P 路 ✓ / stripe1 2×2 ✓）");
    }
    // ── D8 负控：vol1 再拔盘 5 ⇒ 3 缺 ⇒ OFFLINE ──
    {
        raid.member_fail(1, 5);
        std::vector<uint8_t> x(512, 0);
        sc.chk(raid.vol[1].state == VOL_OFFLINE && !raid.vdisk_read(1, 0, x.data(), 1) &&
               !raid.vdisk_write(1, 0, x.data(), 1),
               "D8 负控：3 缺 ⇒ OFFLINE ⇒ 读写皆拒 ✓");
    }
    return sc.verdict("TB_RAID_R6");
}
