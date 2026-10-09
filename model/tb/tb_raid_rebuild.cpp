//============================================================================
// tb_raid_rebuild.cpp — M5 判据台 A：重建引擎  [2026-10-09 建]
//----------------------------------------------------------------------------
//   判据（**预登记** ✓；出处 = `内部内部开发计划` §3 M5 + arch §4.13 Rebuild ✓）：
//   E1 R5 基线：写 [0,768)（2 全条带 ✓ `c_full==2` ✓）⇒ 读回 == 期望 ✓ + 两行 ⊕恒等式 ✓
//   E2 换盘：拔盘1 ⇒ 降级写 [100,120) ⇒ **wipe 盘1 全盘 0xAA**（= 新盘语义 ✓）
//      ⇒ `member_insert` ⇒ 卷/盘 **REBUILDING** ✓
//   E3 **checkpoint 语义**：`rebuild_step`×1 ⇒ true ✓ `rebuild_scan==1` ✓ 卷仍 REBUILDING ✓；
//      **中途读** [0,768) == 期望 ✓（已建行直用 + 未建行重建 ✓ **无陈旧泄漏** ✓）
//   E4 续跑完成：循环 ≤4 ⇒ 卷/盘 ONLINE ✓ `rb_scan==2` ✓ `c_rb==256` 块 ✓
//      **drive1 两行单元逐字节 == 期望片** ✓（重建内容正确 ✓ 核心判据 ✓）；
//      读回 ✓ 恒等式 ✓ 再调 `rebuild_step` ⇒ false ✓
//   E5 **R6 双盘重建**（vol1）：写 2 行（整行 + 半行 ✓）⇒ 拔 {1,2} ⇒ wipe 1,2 ⇒ insert×2
//      ⇒ 循环至完成 ✓：读回 == 期望 ✓；drive1 两行 == 期望片 ✓；drive2 stripe0 == 期望片 ✓、
//      stripe1 == **独立 Q**（TB exp/log 乘法由数据现算 ✓）；两行 P/Q 恒等式 ✓ `c_rb==512` ✓
//   E6 负控：ONLINE 下 `rebuild_step` ⇒ false ✓；拔盘（DEGRADED）⇒ 仍 false 且状态不变 ✓
//      （不误推进非重建态 ✓）
//============================================================================
#include <cstdio>
#include <cstring>
#include <vector>
#include "raid_tb_common.h"

// ── 独立 GF(2⁸)（exp/log 表 ✓ 与模型位算法不同算法 ✓）──
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
};

static bool xor_all_zero(std::vector<std::vector<uint8_t>> &units) {
    std::vector<uint8_t> acc(units[0].size(), 0);
    for (auto &u : units) for (size_t i = 0; i < acc.size(); i++) acc[i] ^= u[i];
    for (auto b : acc) if (b) return false;
    return true;
}

// ── 独立 P/Q 恒等式（由成员现读字节重算比对 ✓；Q 系数 2^k 用 exp/log 乘法独立算 ✓）──
static bool pq_identity(const RaidMemBackend &bk, unsigned v, uint64_t stripe,
                        unsigned n, unsigned nd, uint64_t ss, const GFTable &gf)
{
    unsigned pP = (unsigned)((n - 1) - (stripe % n));
    unsigned pQ = (unsigned)((n - 2) - ((stripe + 1) % n));
    std::vector<uint8_t> P(ss, 0), Q(ss, 0);
    unsigned k = 0;
    for (unsigned d = 0; d < n; d++) {
        if (d == pP || d == pQ) continue;
        uint8_t g = 1; for (unsigned j = 0; j < k; j++) g = gf.mul(g, 2);
        const uint8_t *u = &bk.mem[v][d][stripe * ss];
        for (size_t i = 0; i < ss; i++) { P[i] ^= u[i]; Q[i] ^= gf.mul(g, u[i]); }
        k++;
    }
    return std::memcmp(&bk.mem[v][pP][stripe * ss], P.data(), ss) == 0 &&
           std::memcmp(&bk.mem[v][pQ][stripe * ss], Q.data(), ss) == 0;
}

int sc_main(int argc, char **argv)
{
    (void)argc; (void)argv;
    RaidTbScore sc;
    printf("=== tb_raid_rebuild：重建引擎（R5 单盘 / R6 双盘 / 镜像 ✓）M5 ===\n");

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
    GFTable gf;
    raid.vol_config(0, RAID_LVL_R5, 4, SS, 768);          // R5：行 = 384 块 ⇒ 2 行 ✓
    raid.vol_config(1, RAID_LVL_R6, 6, SS, 768);          // R6：行 = 512 块 ⇒ 2 行 ✓

    std::vector<uint8_t> vexp((size_t)768 * 512, 0);      // vol0 期望 ✓
    std::vector<uint8_t> vexp1((size_t)768 * 512, 0);     // vol1 期望 ✓
    auto wr = [&](unsigned v, uint64_t lba, const std::vector<uint8_t> &b) {
        return raid.vdisk_write(v, lba, b.data(), (unsigned)(b.size() / 512));
    };
    auto rd_eq = [&](unsigned v, uint64_t lba, unsigned nblk, const uint8_t *ex) {
        std::vector<uint8_t> r((size_t)nblk * 512, 0);
        bool ok = raid.vdisk_read(v, lba, r.data(), nblk);
        return ok && std::memcmp(r.data(), ex, (size_t)nblk * 512) == 0;
    };
    auto mem_eq = [&](unsigned v, unsigned d, uint64_t mem_off, const uint8_t *ex, size_t len) {
        return std::memcmp(&bk.mem[v][d][mem_off], ex, len) == 0;
    };

    // ── E1 R5 基线 ──
    std::vector<uint8_t> w1((size_t)768 * 512);
    {
        pat_fill(w1.data(), w1.size(), 0xE1);
        bool okw = wr(0, 0, w1);
        std::memcpy(vexp.data(), w1.data(), w1.size());
        std::vector<std::vector<uint8_t>> r0 = {
            std::vector<uint8_t>(&bk.mem[0][0][0], &bk.mem[0][0][0] + SS),
            std::vector<uint8_t>(&bk.mem[0][1][0], &bk.mem[0][1][0] + SS),
            std::vector<uint8_t>(&bk.mem[0][2][0], &bk.mem[0][2][0] + SS),
            std::vector<uint8_t>(&bk.mem[0][3][0], &bk.mem[0][3][0] + SS) };
        std::vector<std::vector<uint8_t>> r1 = {
            std::vector<uint8_t>(&bk.mem[0][0][SS], &bk.mem[0][0][SS] + SS),
            std::vector<uint8_t>(&bk.mem[0][1][SS], &bk.mem[0][1][SS] + SS),
            std::vector<uint8_t>(&bk.mem[0][2][SS], &bk.mem[0][2][SS] + SS),
            std::vector<uint8_t>(&bk.mem[0][3][SS], &bk.mem[0][3][SS] + SS) };
        sc.chk(okw && raid.cnt_full() == 2 && rd_eq(0, 0, 768, vexp.data()) &&
               xor_all_zero(r0) && xor_all_zero(r1),
               "E1 R5 基线：2 全条带 ✓ 读回 ✓ 两行恒等式 ✓");
    }
    // ── E2 降级写 + 换盘（wipe + insert）──
    {
        std::vector<uint8_t> w2((size_t)20 * 512); pat_fill(w2.data(), w2.size(), 0xE2);
        raid.member_fail(0, 1);
        bool okw = wr(0, 100, w2);
        std::memcpy(&vexp[(size_t)100 * 512], w2.data(), w2.size());
        std::memset(bk.mem[0][1].data(), 0xAA, CAP);       // 新盘 = 全垃圾 ✓
        raid.member_insert(0, 1);
        sc.chk(okw && raid.vol[0].state == VOL_REBUILDING &&
               raid.vol[0].m[1].state == DRV_REBUILDING,
               "E2 换盘：降级写 ✓ wipe 盘1 ✓ insert ⇒ 卷/盘 REBUILDING ✓");
    }
    // ── E3 checkpoint 单步 + 中途读 ──
    {
        bool p1 = raid.rebuild_step(0);
        bool scan1 = (raid.rebuild_lba(0) == 1);
        bool still = (raid.vol[0].state == VOL_REBUILDING);
        sc.chk(p1 && scan1 && still && rd_eq(0, 0, 768, vexp.data()),
               "E3 checkpoint：step×1 ⇒ true/rb_scan=1/仍 REBUILDING ✓ 中途读 == 期望 ✓ 无陈旧泄漏 ✓");
    }
    // ── E4 续跑完成 + 重建内容对拍 ──
    {
        int guard = 0;
        while (raid.vol[0].state == VOL_REBUILDING && guard++ < 4) raid.rebuild_step(0);
        bool online = (raid.vol[0].state == VOL_ONLINE) && (raid.vol[0].m[1].state == DRV_ONLINE);
        bool scan2 = (raid.rebuild_lba(0) == 2);
        bool rb256 = (raid.cnt_rb() == 256);
        bool d1ok = mem_eq(0, 1, 0, &vexp[(size_t)(0 * 384 + 128) * 512], SS) &&
                    mem_eq(0, 1, SS, &vexp[(size_t)(1 * 384 + 128) * 512], SS);
        bool nore = !raid.rebuild_step(0);
        sc.chk(online && scan2 && rb256 && d1ok && nore && rd_eq(0, 0, 768, vexp.data()),
               "E4 完成：ONLINE ✓ rb_scan=2 ✓ c_rb=256 ✓ drive1 两行 == 期望片 ✓ 读回 ✓ 收敛 ✓");
    }
    // ── E5 R6 双盘重建 ──
    {
        std::vector<uint8_t> a((size_t)512 * 512), b((size_t)256 * 512);
        pat_fill(a.data(), a.size(), 0xE5); pat_fill(b.data(), b.size(), 0xE6);
        bool okw = wr(1, 0, a) && wr(1, 512, b);
        std::memcpy(&vexp1[0], a.data(), a.size());
        std::memcpy(&vexp1[(size_t)512 * 512], b.data(), b.size());
        raid.member_fail(1, 1); raid.member_fail(1, 2);
        std::memset(bk.mem[1][1].data(), 0xAA, CAP);
        std::memset(bk.mem[1][2].data(), 0xAA, CAP);
        raid.member_insert(1, 1); raid.member_insert(1, 2);
        uint32_t rb0 = raid.cnt_rb();                      // 全局累计 ✓ 取增量 ✓
        int guard = 0;
        while (raid.vol[1].state == VOL_REBUILDING && guard++ < 16) raid.rebuild_step(1);
        // drive1：两行 k=1 ⇒ 期望片 ✓
        bool d1ok = mem_eq(1, 1, 0, &vexp1[(size_t)(0 * 512 + 128) * 512], SS) &&
                    mem_eq(1, 1, SS, &vexp1[(size_t)(1 * 512 + 128) * 512], SS);
        // drive2：stripe0 k=2 期望片 ✓；stripe1 = Q（独立 exp/log 乘法现算 ✓）
        //   ⚠ 用**整行缓冲**（512 块 ✓ 未写段为 0 ✓）—— 直接从 vexp1 取 k=2/3 会**越界** ✗
        //     （vexp1 只到 768 块 ⇒ k=2 起在尾后 ⇒ 读堆垃圾 ✗ 本轮实踩 ✓）
        bool d2a = mem_eq(1, 2, 0, &vexp1[(size_t)(0 * 512 + 256) * 512], SS);
        std::vector<uint8_t> row1((size_t)512 * 512, 0);
        std::memcpy(&row1[0], &vexp1[(size_t)512 * 512], (size_t)256 * 512);
        std::vector<uint8_t> qexp(SS, 0);
        for (unsigned k = 0; k < 4; k++) {
            uint8_t g = 1; for (unsigned j = 0; j < k; j++) g = gf.mul(g, 2);
            const uint8_t *u = &row1[(size_t)k * 128 * 512];
            for (size_t i = 0; i < SS; i++) qexp[i] ^= gf.mul(g, u[i]);
        }
        bool d2b = mem_eq(1, 2, SS, qexp.data(), SS);
        bool done = (raid.vol[1].state == VOL_ONLINE) && (guard <= 6) && (raid.cnt_rb() - rb0 == 512);
        bool pq = pq_identity(bk, 1, 0, 6, 4, SS, gf) && pq_identity(bk, 1, 1, 6, 4, SS, gf);
        bool rdo = rd_eq(1, 0, 768, vexp1.data());
        if (!d2b) {
            size_t i0 = 0;
            while (i0 < SS && bk.mem[1][2][SS + i0] == qexp[i0]) i0++;
            printf("      [E5 diff] Q @%zu member=%02x exp=%02x\n", i0, bk.mem[1][2][SS + i0], qexp[i0]);
        }
        sc.chk(okw && d1ok && d2a && d2b && done && pq && rdo,
               "E5 R6 双盘：drive1 期望片 ✓ drive2 数据片+独立 Q ✓ 完成(c_rb=512) ✓ P/Q 恒等式 ✓ 读回 ✓");
    }
    // ── E6 负控 ──
    {
        bool idle = !raid.rebuild_step(0);                  // vol0 已 ONLINE ✓
        raid.member_fail(0, 0);
        bool deg = (raid.vol[0].state == VOL_DEGRADED);
        bool nogo = !raid.rebuild_step(0);
        bool stay = (raid.vol[0].state == VOL_DEGRADED);
        sc.chk(idle && deg && nogo && stay,
               "E6 负控：ONLINE⇒false ✓ DEGRADED⇒false 且状态不变 ✓（不误推进 ✓）");
    }
    return sc.verdict("TB_RAID_REBUILD");
}
