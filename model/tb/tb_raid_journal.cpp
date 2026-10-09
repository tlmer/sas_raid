//============================================================================
// tb_raid_journal.cpp — M5 判据台 B：写洞日志（Write-Hole Journal）  [2026-10-09 建]
//----------------------------------------------------------------------------
//   判据（**预登记** ✓；出处 = `内部内部开发计划` §3 M5 + arch §4.13 Write-Hole ✓）：
//   F1 **正常路径无残留**：全条带写 + RMW 写 ⇒ `journal_count()==0` ✓（写毕即清 ✓）
//   F2 **失败写留痕**：拔盘 1 ⇒ 整行写 [0,384) 两次 ⇒ **均拒** ✓ 且 count 1→2 ✓
//      （unit0 落盘并清 ✓ / unit1 写前记下、因失败盘拒 ⇒ 留痕 ✓）；卷仍 DEGRADED ✓
//      部分写内容 == 期望 ✓（无失配半写 ✓）
//   F3 **重放幂等**：全盘快照 ⇒ `journal_replay`×2 ⇒ count 0 ✓✓ 且**数据逐字节不变** ✓
//   F4 **重放后一致性**：读 [0,768) == 期望 ✓ + 行0 恒等式（盘1 用模型重建读 ✓）
//   F5 **重放后可继续工作**：写 [300,320)（在线数据盘单元 ✓）⇒ 成功 ✓ count 保持 0 ✓ 回读 ✓
//   ⛔ 环满（256 项）行为 = 简化"满则丢" ✓ —— 本台不测 ✗（记为扩展点 ✓）
//============================================================================
#include <cstdio>
#include <cstring>
#include <vector>
#include "raid_tb_common.h"

static bool xor_all_zero(std::vector<std::vector<uint8_t>> &units) {
    std::vector<uint8_t> acc(units[0].size(), 0);
    for (auto &u : units) for (size_t i = 0; i < acc.size(); i++) acc[i] ^= u[i];
    for (auto b : acc) if (b) return false;
    return true;
}

int sc_main(int argc, char **argv)
{
    (void)argc; (void)argv;
    RaidTbScore sc;
    printf("=== tb_raid_journal：写洞日志（留痕/幂等重放/继续工作 ✓）M5 ===\n");

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
    raid.vol_config(0, RAID_LVL_R5, 4, SS, 768);

    std::vector<uint8_t> vexp((size_t)768 * 512, 0);
    auto wr = [&](uint64_t lba, const std::vector<uint8_t> &b) {
        return raid.vdisk_write(0, lba, b.data(), (unsigned)(b.size() / 512));
    };
    auto rd_eq = [&](uint64_t lba, unsigned nblk, const uint8_t *ex) {
        std::vector<uint8_t> r((size_t)nblk * 512, 0);
        bool ok = raid.vdisk_read(0, lba, r.data(), nblk);
        return ok && std::memcmp(r.data(), ex, (size_t)nblk * 512) == 0;
    };

    // ── F1 正常路径无残留 ──
    {
        std::vector<uint8_t> w1((size_t)384 * 512); pat_fill(w1.data(), w1.size(), 0xF1);
        std::vector<uint8_t> w2((size_t)20 * 512);  pat_fill(w2.data(), w2.size(), 0xF2);
        bool okw = wr(0, w1) && wr(100, w2);
        std::memcpy(&vexp[0], w1.data(), w1.size());
        std::memcpy(&vexp[(size_t)100 * 512], w2.data(), w2.size());
        sc.chk(okw && raid.journal_count() == 0 && rd_eq(0, 768, vexp.data()),
               "F1 正常路径：全条带+RMW 写后 count==0 ✓（写毕即清 ✓）读回 ✓");
    }
    // ── F2 失败写留痕 1 → 2 ──
    {
        std::vector<uint8_t> w3((size_t)384 * 512); pat_fill(w3.data(), w3.size(), 0xF3);
        raid.member_fail(0, 1);
        bool r1 = !wr(0, w3);
        bool c1 = (raid.journal_count() == 1);
        bool r2 = !wr(0, w3);
        bool c2 = (raid.journal_count() == 2);
        std::memcpy(&vexp[0], w3.data(), (size_t)128 * 512);   // unit0 每次尝试都落盘 ✓
        sc.chk(r1 && c1 && r2 && c2 && raid.vol[0].state == VOL_DEGRADED &&
               rd_eq(0, 768, vexp.data()),
               "F2 失败写留痕：两次均拒 ✓ count 1→2 ✓ DEGRADED ✓ 部分写 == 期望（无失配半写 ✓）");
    }
    // ── F3 重放幂等 + 数据不变 ──
    {
        std::vector<std::vector<uint8_t>> snap;
        for (unsigned d = 0; d < 4; d++) snap.push_back(bk.mem[0][d]);
        raid.journal_replay(0);
        bool c0 = (raid.journal_count() == 0);
        raid.journal_replay(0);
        bool c0b = (raid.journal_count() == 0);
        bool same = true;
        for (unsigned d = 0; d < 4; d++)
            if (std::memcmp(snap[d].data(), bk.mem[0][d].data(), CAP) != 0) same = false;
        sc.chk(c0 && c0b && same, "F3 重放幂等：replay×2 ⇒ count 0 ✓✓ 数据逐字节不变 ✓");
    }
    // ── F4 重放后一致性（行0 恒等式：盘1 用模型重建读 ✓）──
    {
        std::vector<uint8_t> d1(SS, 0);
        bool okr = raid.vdisk_read(0, 128, d1.data(), 128);
        std::vector<std::vector<uint8_t>> us = {
            std::vector<uint8_t>(&bk.mem[0][0][0], &bk.mem[0][0][0] + SS), d1,
            std::vector<uint8_t>(&bk.mem[0][2][0], &bk.mem[0][2][0] + SS),
            std::vector<uint8_t>(&bk.mem[0][3][0], &bk.mem[0][3][0] + SS) };
        sc.chk(okr && rd_eq(0, 768, vexp.data()) && xor_all_zero(us),
               "F4 重放后一致性：读全卷 == 期望 ✓ 行0 恒等式（含重建读 ✓）✓");
    }
    // ── F5 重放后继续工作 ──
    {
        std::vector<uint8_t> w5((size_t)20 * 512); pat_fill(w5.data(), w5.size(), 0xF5);
        uint32_t r0 = raid.cnt_rmw();
        bool okw = wr(300, w5);
        std::memcpy(&vexp[(size_t)300 * 512], w5.data(), w5.size());
        sc.chk(okw && raid.journal_count() == 0 && (raid.cnt_rmw() - r0 == 1) &&
               rd_eq(300, 20, &vexp[(size_t)300 * 512]),
               "F5 继续工作：重放后再写成功 ✓ count 保持 0 ✓ 回读 ✓（c_rmw +1 ✓）");
    }
    return sc.verdict("TB_RAID_JOURNAL");
}
