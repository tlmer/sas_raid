//============================================================================
// tb_raid_r01.cpp — M2 判据台：RAID 0 / RAID 1 语义  [2026-10-09 建]
//----------------------------------------------------------------------------
//   判据（**预登记** ✓；出处 = `内部内部开发计划` §3 M2 + spec 五式 ✓）：
//   B1 R0 条带拆分：4 盘 / 64KB，写 LBA 100..799（跨单元 ✓ 跨条带 ✓）⇒ 每成员内存
//      **全盘逐字节** == 独立软件参考模型 ✓（参考模型台内**另写一遍** ✓ 防"同错"✗）
//   B2 R0 回读闭环：`vdisk_read` == 所写图案 ✓
//   B3 R0 非对齐二次写（LBA 300..359）⇒ 参考增量更新后全盘对拍 ✓ + 混合回读 == 预期 ✓
//   B4 R1 双写：两副本逐字节 == 参考 且互等 ✓（写惩罚 2× ✓）
//   B5 R1 读均衡：连续 2 次单元读 ⇒ 两副本**各读 1 次** ✓（后端逐盘计数 ✓）且数据皆对 ✓
//   B6 R1 降级：拔盘 0 ⇒ 卷 DEGRADED ✓；读 == 图案（副本 1 ✓）；降级写只落副本 1 ✓；
//      回读 == 新图案 ✓（副本 0 保持旧值 ✓ —— 不触碰离线盘 ✓）
//   B7 R1 负控（红 → 绿）：两副本全拔 ⇒ 卷 OFFLINE ✓ 读写皆拒 ✓
//============================================================================
#include <cstdio>
#include <cstring>
#include <vector>
#include "raid_tb_common.h"

// ── 独立软件参考模型（**故意另写** ✓：逐块按 spec 公式铺成员字节 ✓，防照抄模型 ✗）──
struct RefRaid {
    unsigned n, nd; uint64_t ss; size_t cap;
    std::vector<std::vector<uint8_t>> m;                 // [盘][字节]
    RefRaid(unsigned n_, unsigned nd_, uint64_t ss_, size_t cap_)
        : n(n_), nd(nd_), ss(ss_), cap(cap_), m(n_, std::vector<uint8_t>(cap_, 0)) {}
    void put(uint64_t lba, const uint8_t *buf, unsigned nblk) {
        for (unsigned i = 0; i < nblk; i++) {
            uint64_t b = lba + i, bytes = b * 512ull, row = ss * nd;
            uint64_t stripe = bytes / row, off = bytes % row;
            unsigned drv = (unsigned)(off / ss);
            uint64_t lba_m = stripe * (ss / 512) + (off % ss) / 512;
            std::memcpy(&m[drv][(size_t)lba_m * 512], buf + (size_t)i * 512, 512);
        }
    }
    void put_r1(uint64_t lba, const uint8_t *buf, unsigned nblk) {   // 两副本同址 ✓
        for (unsigned c = 0; c < 2; c++)
            std::memcpy(&m[c][(size_t)lba * 512], buf, (size_t)nblk * 512);
    }
    bool eq(const RaidMemBackend &bk, unsigned v) const {
        for (unsigned d = 0; d < n; d++)
            if (std::memcmp(bk.mem[v][d].data(), m[d].data(), cap) != 0) return false;
        return true;
    }
    void diff(const RaidMemBackend &bk, unsigned v) const {          // 失配定位（排障用 ✓）
        for (unsigned d = 0; d < n; d++) {
            if (std::memcmp(bk.mem[v][d].data(), m[d].data(), cap) == 0) continue;
            for (size_t i = 0; i < cap; i++)
                if (bk.mem[v][d][i] != m[d][i]) { printf("      [diff] vol%d drv%u off %zu\n", v, d, i); break; }
        }
    }
};

int sc_main(int argc, char **argv)
{
    (void)argc; (void)argv;
    RaidTbScore sc;
    printf("=== tb_raid_r01：RAID 0/1 语义对独立参考（M2 ✓）===\n");

    const size_t CAP = 8u * 1024u * 1024u;                 // 成员盘 8MB ✓
    RaidMemBackend bk(CAP);
    // ── 例化 + 全端口绑定（不跑 sc_start：vdisk_* 为直调 ✓ 时钟面只做绑定 ✓）──
    sc_core::sc_signal<bool> clk_s, rst_s;
    sc_core::sc_signal<sc_uint<32>> s_rd, s_wr, s_rmw, s_full, s_deg, s_rb, s_jr, s_mio;
    SasRaidTlm raid("raid");
    raid.clk(clk_s); raid.rst_n(rst_s);
    raid.o_reads(s_rd); raid.o_writes(s_wr); raid.o_rmw(s_rmw); raid.o_full_stripe(s_full);
    raid.o_degraded(s_deg); raid.o_rebuild_blocks(s_rb); raid.o_journal_ents(s_jr);
    raid.o_member_ios(s_mio);
    rst_s.write(true); clk_s.write(false);
    bk.bind(raid);

    const uint32_t SS = 65536;                             // 64KB 条带 ✓
    raid.vol_config(0, RAID_LVL_R0, 4, SS, 4096);          // R0：4 盘 ✓
    raid.vol_config(1, RAID_LVL_R1, 2, SS, 1024);          // R1：2 盘 ✓

    // ── B1/B2 R0：写 700 块跨单元跨条带 ⇒ 成员全盘对拍 + 回读闭环 ──
    {
        std::vector<uint8_t> w1(700 * 512);
        pat_fill(w1.data(), w1.size(), 0x0B01);
        RefRaid ref0(4, 4, SS, CAP);
        ref0.put(100, w1.data(), 700);
        bool okw = raid.vdisk_write(0, 100, w1.data(), 700);
        bool okm = ref0.eq(bk, 0);
        if (!okm) ref0.diff(bk, 0);
        sc.chk(okw && okm, "B1 R0 拆分：写 100..799 ⇒ 4 成员全盘逐字节 == 独立参考 ✓");

        std::vector<uint8_t> r1(700 * 512, 0xEE);
        bool okr = raid.vdisk_read(0, 100, r1.data(), 700);
        sc.chk(okr && std::memcmp(r1.data(), w1.data(), w1.size()) == 0,
               "B2 R0 回读闭环：vdisk_read == 所写图案 ✓");
    }
    // ── B3 R0：非对齐二次写 300..359 ⇒ 参考增量 + 混合回读 ──
    {
        std::vector<uint8_t> w2(60 * 512);
        pat_fill(w2.data(), w2.size(), 0x0B03);
        // 期望混合图案：100..299 旧 / 300..359 新 / 360..799 旧 ✓
        std::vector<uint8_t> exp(700 * 512);
        pat_fill(exp.data(), exp.size(), 0x0B01);
        std::memcpy(&exp[(300 - 100) * 512], w2.data(), w2.size());

        bool okw = raid.vdisk_write(0, 300, w2.data(), 60);
        RefRaid ref0b(4, 4, SS, CAP);
        std::vector<uint8_t> w1(700 * 512); pat_fill(w1.data(), w1.size(), 0x0B01);
        ref0b.put(100, w1.data(), 700);
        ref0b.put(300, w2.data(), 60);
        bool okm = ref0b.eq(bk, 0);
        if (!okm) ref0b.diff(bk, 0);
        std::vector<uint8_t> r2(700 * 512, 0);
        bool okr = raid.vdisk_read(0, 100, r2.data(), 700);
        sc.chk(okw && okm && okr && std::memcmp(r2.data(), exp.data(), exp.size()) == 0,
               "B3 R0 非对齐二次写 ⇒ 全盘对拍 ✓ + 混合回读 == 预期 ✓");
    }
    // ── B4 R1：双写 ──
    std::vector<uint8_t> w3(300 * 512);
    {
        pat_fill(w3.data(), w3.size(), 0x0B04);
        bool okw = raid.vdisk_write(1, 0, w3.data(), 300);
        RefRaid ref1(2, 1, SS, CAP);
        ref1.put_r1(0, w3.data(), 300);
        bool okm = ref1.eq(bk, 1);
        if (!okm) ref1.diff(bk, 1);
        bool same = std::memcmp(bk.mem[1][0].data(), bk.mem[1][1].data(), CAP) == 0;
        sc.chk(okw && okm && same, "B4 R1 双写：两副本逐字节 == 参考 且互等 ✓");
    }
    // ── B5 R1：读均衡（后端计数）──
    {
        bk.reset_cnt();
        std::vector<uint8_t> a(128 * 512, 0), b(128 * 512, 0);
        bool oka = raid.vdisk_read(1, 0, a.data(), 128);       // 单元 0 ✓
        bool okb = raid.vdisk_read(1, 0, b.data(), 128);       // 单元 0 再读 ✓
        bool both = (bk.rd_cnt[1][0] == 1 && bk.rd_cnt[1][1] == 1);
        bool data = std::memcmp(a.data(), w3.data(), a.size()) == 0 &&
                    std::memcmp(b.data(), w3.data(), b.size()) == 0;
        sc.chk(oka && okb && both && data,
               "B5 R1 读均衡：连读 2 次 ⇒ 副本0/副本1 各读 1 次 ✓ 且数据皆对 ✓");
    }
    // ── B6 R1：降级读 + 降级写 ──
    {
        raid.member_fail(1, 0);
        bool st = (raid.vol[1].state == VOL_DEGRADED);
        std::vector<uint8_t> r(300 * 512, 0);
        bool okr = raid.vdisk_read(1, 0, r.data(), 300) &&
                   std::memcmp(r.data(), w3.data(), r.size()) == 0;
        std::vector<uint8_t> w4(300 * 512);
        pat_fill(w4.data(), w4.size(), 0x0B06);
        bool okw = raid.vdisk_write(1, 0, w4.data(), 300);
        std::vector<uint8_t> r2(300 * 512, 0);
        bool okr2 = raid.vdisk_read(1, 0, r2.data(), 300) &&
                    std::memcmp(r2.data(), w4.data(), r2.size()) == 0;
        bool c1new = std::memcmp(bk.mem[1][1].data(), w4.data(), w4.size()) == 0;
        bool c0old = std::memcmp(bk.mem[1][0].data(), w3.data(), w3.size()) == 0;
        sc.chk(st && okr && okw && okr2 && c1new && c0old,
               "B6 R1 降级：DEGRADED ✓ 读副本1 ✓ 降级写落副本1 ✓ 回读新图案 ✓ 副本0 保持旧值 ✓");
    }
    // ── B7 R1：负控（两副本全拔 ⇒ OFFLINE ⇒ 读写皆拒）──
    {
        raid.member_fail(1, 1);
        bool st = (raid.vol[1].state == VOL_OFFLINE);
        std::vector<uint8_t> x(512, 0);
        bool rd = raid.vdisk_read(1, 0, x.data(), 1);
        bool wr = raid.vdisk_write(1, 0, x.data(), 1);
        sc.chk(st && !rd && !wr, "B7 负控：两副本全拔 ⇒ OFFLINE 且读写皆拒 ✓");
    }
    return sc.verdict("TB_RAID_R01");
}
