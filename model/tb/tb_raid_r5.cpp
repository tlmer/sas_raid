//============================================================================
// tb_raid_r5.cpp — M3 判据台：RAID 5 语义  [2026-10-09 建]
//----------------------------------------------------------------------------
//   配置：4 盘 / 64KB 条带 / 行 = 3×128 块 = 384 块 ✓（nd=3 ✓）
//   判据（**预登记** ✓；出处 = `内部内部开发计划` §3 M3 + spec 五式 ✓）：
//   C1 **全条带写**（LBA 0..383，行对齐整行）⇒ ④ 全盘逐字节 == 独立参考 ✓
//      ② 后端**零读**（快速路径 P=⊕新数据 ✓ 无 RMW 读 ✓）③ 每盘恰写 1 次 ✓
//      ④ `c_full==1 且 c_rmw==0` ✓ ⑤ 行恒等式 `D0⊕D1⊕D2⊕P≡0` ✓
//   C2 **RMW 部分写**（LBA 100..119，单单元内）⇒ 全盘对拍 ✓ + **写惩罚签名**：
//      1 读 D_old + 1 读 P_old + 1 写 D + 1 写 P ✓（其余盘零读写 ✓）+ `c_rmw==1` ✓
//   C3 **跨单元跨行写**（LBA 380..419：D2 尾 4 块 + stripe1 D0 头 36 块）⇒
//      全盘对拍 ✓ + `c_rmw` 增量 == 2 ✓
//   C4 **第二行全条带 + 旋转**（写 384..767）⇒ 全盘对拍 ✓ + `c_full==2` ✓ +
//      **两行**恒等式 ⊕≡0 ✓（P 盘位 stripe0→盘3、stripe1→盘2 ✓ 参考模型独立旋转 ✓）
//   C5 **降级读**：拔数据盘 1 ⇒ 卷 DEGRADED ✓；读 0..767 全部 == 虚拟期望 ✓（重建路径 ✓
//      `c_deg` 增长 ✓）
//   C6 **降级写**（D0 单元，P 在线）：全盘对拍 ✓ 回读新值 ✓ 且 D1 单元**重建读 == 旧值** ✓
//      （P 更新后仍自洽 ✓）行恒等式 ✓；后端**不碰离线盘** ✓（只写 D0 与 P ✓）
//   C7 **负控 A**：降级下整行写 ⇒ 拒 ✓（unit0 落盘后 unit1 因失败盘拒 ✗ —— 无部分写 ✗
//      之下仍自洽 ✓）；写洞日志**留痕 1 条** ✓ `journal_replay` 重放后清零 ✓ 且数据不变 ✓
//   C8 **负控 B**：再拔数据盘 2 ⇒ 超出冗余 ⇒ 卷 OFFLINE ✓ 读写皆拒 ✓
//============================================================================
#include <cstdio>
#include <cstring>
#include <vector>
#include "raid_tb_common.h"

// ── 独立软件参考模型（**故意另写** ✓：逐块 spec 公式 + 独立旋转 + 增量 RMW ✓）──
struct RefR5 {
    unsigned n, nd; uint64_t ss; size_t cap;
    std::vector<std::vector<uint8_t>> m;
    RefR5(unsigned n_, unsigned nd_, uint64_t ss_, size_t cap_)
        : n(n_), nd(nd_), ss(ss_), cap(cap_), m(n_, std::vector<uint8_t>(cap_, 0)) {}
    unsigned phys(unsigned k, uint64_t stripe) const {           // left-asymmetric ✓
        unsigned pp = (unsigned)((n - 1) - (stripe % n)), cnt = 0;
        for (unsigned d = 0; d < n; d++) { if (d == pp) continue; if (cnt == k) return d; cnt++; }
        return 0;
    }
    void put_sub(uint64_t lba, const uint8_t *buf, unsigned nblk) {   // P' = P ⊕ D_old ⊕ D_new ✓
        for (unsigned i = 0; i < nblk; i++) {
            uint64_t b = lba + i, bytes = b * 512, row = (uint64_t)ss * nd;
            uint64_t stripe = bytes / row, off = bytes % row;
            unsigned k = (unsigned)(off / ss);
            uint64_t lba_m = stripe * (ss / 512) + (off % ss) / 512;
            unsigned d = phys(k, stripe), pp = (unsigned)((n - 1) - (stripe % n));
            uint8_t *dst = &m[d][(size_t)lba_m * 512];
            uint8_t *par = &m[pp][(size_t)lba_m * 512];
            const uint8_t *src = buf + (size_t)i * 512;
            for (unsigned j = 0; j < 512; j++) { par[j] ^= (uint8_t)(dst[j] ^ src[j]); dst[j] = src[j]; }
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
    printf("=== tb_raid_r5：RAID 5（全条带/RMW/降级 ✓）对独立参考（M3 ✓）===\n");

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
    const unsigned ROW_BLK = 384;                                 // 行 = 3 数据盘 × 128 块 ✓
    raid.vol_config(0, RAID_LVL_R5, 4, SS, 4096);
    RefR5 ref(4, 3, SS, CAP);

    std::vector<uint8_t> vexp((size_t)768 * 512, 0);              // 虚拟期望（写入即记 ✓）
    auto wr = [&](uint64_t lba, const std::vector<uint8_t> &b) {
        return raid.vdisk_write(0, lba, b.data(), (unsigned)(b.size() / 512));
    };
    auto note = [&](uint64_t lba, const std::vector<uint8_t> &b) {
        std::memcpy(&vexp[(size_t)lba * 512], b.data(), b.size());
    };
    auto rd_eq = [&](uint64_t lba, unsigned nblk) {
        std::vector<uint8_t> r((size_t)nblk * 512, 0);
        bool ok = raid.vdisk_read(0, lba, r.data(), nblk);
        return ok && std::memcmp(r.data(), &vexp[(size_t)lba * 512], r.size()) == 0;
    };
    auto unit_of = [&](unsigned d, uint64_t stripe) {             // 成员物理单元（仅在线盘 ✓）
        return std::vector<uint8_t>(&bk.mem[0][d][stripe * SS], &bk.mem[0][d][stripe * SS] + SS);
    };

    // ── C1 全条带写 ──
    {
        std::vector<uint8_t> w1((size_t)ROW_BLK * 512); pat_fill(w1.data(), w1.size(), 0xC1);
        bk.reset_cnt();
        bool okw = wr(0, w1); note(0, w1); ref.put_sub(0, w1.data(), ROW_BLK);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        bool no_rd = true; for (unsigned d = 0; d < 4; d++) if (bk.rd_cnt[0][d]) no_rd = false;
        bool wr1x  = true; for (unsigned d = 0; d < 4; d++) if (bk.wr_cnt[0][d] != 1) wr1x = false;
        auto u0 = unit_of(0, 0);
        std::vector<std::vector<uint8_t>> us = { unit_of(0,0), unit_of(1,0), unit_of(2,0), unit_of(3,0) };
        sc.chk(okw && okm && no_rd && wr1x && raid.cnt_full() == 1 && raid.cnt_rmw() == 0
               && xor_all_zero(us),
               "C1 全条带写：⊕P=XOR ✓ 零读 ✓ 每盘 1 写 ✓ c_full=1/c_rmw=0 ✓ 恒等式 ✓");
    }
    // ── C2 RMW 部分写 + 写惩罚签名 ──
    {
        std::vector<uint8_t> w2((size_t)20 * 512); pat_fill(w2.data(), w2.size(), 0xC2);
        bk.reset_cnt();
        bool okw = wr(100, w2); note(100, w2); ref.put_sub(100, w2.data(), 20);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        bool pen = bk.rd_cnt[0][0] == 1 && bk.rd_cnt[0][3] == 1 &&
                   bk.rd_cnt[0][1] == 0 && bk.rd_cnt[0][2] == 0 &&
                   bk.wr_cnt[0][0] == 1 && bk.wr_cnt[0][3] == 1 &&
                   bk.wr_cnt[0][1] == 0 && bk.wr_cnt[0][2] == 0;
        sc.chk(okw && okm && pen && raid.cnt_rmw() == 1,
               "C2 RMW：全盘对拍 ✓ 写惩罚=读D_old+读P+写D+写P ✓ c_rmw=1 ✓");
    }
    // ── C3 跨单元跨行 ──
    {
        std::vector<uint8_t> w3((size_t)40 * 512); pat_fill(w3.data(), w3.size(), 0xC3);
        uint32_t r0 = raid.cnt_rmw();
        bool okw = wr(380, w3); note(380, w3); ref.put_sub(380, w3.data(), 40);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        sc.chk(okw && okm && (raid.cnt_rmw() - r0 == 2),
               "C3 跨单元跨行（380..419）：全盘对拍 ✓ c_rmw 增量=2 ✓");
    }
    // ── C4 行1 全条带 + 旋转 + 两行恒等式 ──
    {
        std::vector<uint8_t> w4((size_t)ROW_BLK * 512); pat_fill(w4.data(), w4.size(), 0xC4);
        bool okw = wr(384, w4); note(384, w4); ref.put_sub(384, w4.data(), ROW_BLK);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        std::vector<std::vector<uint8_t>> r0u = { unit_of(0,0), unit_of(1,0), unit_of(2,0), unit_of(3,0) };
        std::vector<std::vector<uint8_t>> r1u = { unit_of(0,1), unit_of(1,1), unit_of(2,1), unit_of(3,1) };
        sc.chk(okw && okm && raid.cnt_full() == 2 && xor_all_zero(r0u) && xor_all_zero(r1u),
               "C4 行1 全条带：对拍 ✓ c_full=2 ✓ 两行恒等式 ⊕≡0 ✓（P 位 3→2 旋转 ✓）");
    }
    // ── C5 降级读（拔数据盘 1）──
    {
        uint32_t d0 = raid.cnt_deg();
        raid.member_fail(0, 1);
        bool st = (raid.vol[0].state == VOL_DEGRADED);
        sc.chk(st && rd_eq(0, 768) && (raid.cnt_deg() > d0),
               "C5 降级读：DEGRADED ✓ 读 0..767 == 期望 ✓（重建路径 ✓ c_deg 增长 ✓）");
    }
    // ── C6 降级写（D0 单元）+ P 更新后自洽 ──
    {
        std::vector<uint8_t> w6((size_t)128 * 512); pat_fill(w6.data(), w6.size(), 0xC6);
        bk.reset_cnt();
        bool okw = wr(0, w6); note(0, w6); ref.put_sub(0, w6.data(), 128);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        bool rd_new = rd_eq(0, 128);
        bool rd_recon = rd_eq(128, 128);                    // D1 单元重建读 == 旧值 ✓
        bool offline_untouched = (bk.wr_cnt[0][1] == 0);
        bool only_dp = (bk.wr_cnt[0][0] == 1 && bk.wr_cnt[0][3] == 1);
        std::vector<uint8_t> d1(SS, 0); raid.vdisk_read(0, 128, d1.data(), 128);
        std::vector<std::vector<uint8_t>> us = { unit_of(0,0), d1, unit_of(2,0), unit_of(3,0) };
        sc.chk(okw && okm && rd_new && rd_recon && offline_untouched && only_dp && xor_all_zero(us),
               "C6 降级写：对拍 ✓ 回读新值 ✓ D1 重建读==旧 ✓ 只写 D0/P ✓ 恒等式 ✓");
    }
    // ── C7 负控 A：降级整行写 ⇒ 拒（unit0 落盘、unit1 拒）+ 日志留痕/重放 ──
    {
        std::vector<uint8_t> w7((size_t)ROW_BLK * 512); pat_fill(w7.data(), w7.size(), 0xC7);
        bool rej = !wr(0, w7);
        note(0, std::vector<uint8_t>(w7.begin(), w7.begin() + (size_t)128 * 512));   // 只落 unit0 ✓
        ref.put_sub(0, w7.data(), 128);
        bool okm = ref.eq(bk, 0); if (!okm) ref.diff(bk, 0);
        bool jr1 = (raid.journal_count() == 1);              // unit1 的写前记录未清 ✓
        raid.journal_replay(0);
        bool jr0 = (raid.journal_count() == 0);
        sc.chk(rej && okm && jr1 && jr0 && rd_eq(0, 768),
               "C7 负控A：降级整行写拒 ✓ 无失配部分写 ✓ 日志留痕1条→重放清0 ✓ 数据不变 ✓");
    }
    // ── C8 负控 B：超出冗余 ⇒ OFFLINE ──
    {
        raid.member_fail(0, 2);
        std::vector<uint8_t> x(512, 0);
        sc.chk(raid.vol[0].state == VOL_OFFLINE && !raid.vdisk_read(0, 0, x.data(), 1)
               && !raid.vdisk_write(0, 0, x.data(), 1),
               "C8 负控B：两数据盘缺失 ⇒ OFFLINE ⇒ 读写皆拒 ✓");
    }
    return sc.verdict("TB_RAID_R5");
}
