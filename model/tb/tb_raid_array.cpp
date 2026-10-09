//============================================================================
// tb_raid_array.cpp — ★ M6 联合**交互性**台：RAID ⇄ HBA ⇄ link ⇄ HDD ×4
//   [2026-10-09 建；用户令"做各种交互性测试" ✓ 内部开发计划 §3-M6 ✓]
//----------------------------------------------------------------------------
//   结构 = `SasRaidArrayTop`（4 路点对点 SAS 域 ✓ 每成员一路 ✓ 复用 SasPairTop ✓）：
//   每成员 I/O 走**真帧级全栈**：DQ 条目 → HBA 取指 → SSP COMMAND → link → HDD
//   → DATA/RESPONSE → CQE → 读回 ✓（口径与 `tb_pair_bringup` 同源 ✓）
//
//   判据（**预登记** ✓）：
//   A1 **基线全条带写** [0,768)（R5 4 盘 ✓ 2 全条带 ✓）：读回 == 期望 ✓；`c_full==2` ✓；
//      每通道恰 2 次成员 I/O（每盘每行 1 次 ✓ 整行写 = 每成员 1 命令 ✓）
//   A2 **成员存储级对拍**：4 盘 × 2 单元逐字节 == 期望（**经 HDD 存储直读 ✓**；两行的
//      P 由 TB 独立 XOR 现算 ✓ —— 帧级链路落地的**真存储**才算数 ✓）
//   A3 **降级读**（拔盘 1）：读全卷 == 期望 ✓；且 **ch1 帧计数纹丝不动** ✓
//      （o_frames_tx/rx、o_cqes、hdd.o_rx_frames 全零增量 ✓ 帧级证据 ✓）
//   A4 **降级写** [100,120)：读回 ✓；ch1 仍冻结 ✓；`c_rmw==1` ✓（只动 D0 与 P ✓）
//   A5 **换盘重建**：drv1 两个单元 wipe 成 0xAA（= 新盘 ✓）⇒ `member_insert` ⇒
//      `rebuild_step` 循环至完成 ✓：卷 ONLINE ✓ `c_rb` 增量==256 ✓；
//      **drv1 存储 == 期望片** ✓（帧级写入的存储级复核 ✓）；读全卷 == 期望 ✓
//   A6 **账目闭合**（逐通道）：`hba.o_cqes == io_cnt` ✓ `hdd.o_cmds_good == io_cnt` ✓；
//      `o_cmds_err==0` / `o_tag_mismatch==0` / link `dropped==0` ✓（四通道 ✓）
//   A7 **块账**（逐通道）：`hdd.o_lba_read == blk_rd` ✓ `hdd.o_lba_write == blk_wr` ✓
//      （链路往返的块数逐盘对账 ✓）
//   A8 **负控**：再拔盘 2+3（两缺超冗余 ✓）⇒ 卷 OFFLINE ⇒ 读写皆拒 ✓
//      且**四通道帧计数全冻结** ✓（拒绝发生在 RAID 层、不发帧 ✓）
//============================================================================
#include <systemc.h>
#include "sas_raid_array_top.h"
#include "dummy_amba_master.h"
#include "raid_tb_common.h"          // pat_fill ✓（含 sas_raid_tlm.h ✓ 与顶层同源 ✓）
#include <cstdio>
#include <cstring>
#include <vector>

struct TbArr : sc_core::sc_module {
    sc_core::sc_in<bool>  clk;
    sc_core::sc_out<bool> rst_n;
    SasRaidArrayTop *arr = nullptr;
    int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(TbArr);
    TbArr(sc_core::sc_module_name nm) : sc_module(nm), clk("clk"), rst_n("rst_n") {
        SC_THREAD(run);
    }
    void tick(int n = 1) { for (int i = 0; i < n; i++) wait(clk.posedge_event()); }
    void chk(bool ok, const char *name) {
        printf("   %s %s\n", ok ? "✓" : "✗", name);
        if (ok) pass_cnt++; else fail_cnt++;
    }

    // 期望向量（虚拟盘 ✓）
    std::vector<uint8_t> vexp;
    static void xor3(const uint8_t *a, const uint8_t *b, const uint8_t *c, std::vector<uint8_t> &out) {
        out.resize(65536);
        for (size_t i = 0; i < out.size(); i++) out[i] = (uint8_t)(a[i] ^ b[i] ^ c[i]);
    }
    bool drv_unit_eq(unsigned d, uint64_t mem_lba, const uint8_t *ex) {
        const uint8_t *got = arr->chan(d).hdd.lba_ptr(mem_lba);
        size_t i = 0;
        while (i < 65536 && got[i] == ex[i]) i++;
        if (i < 65536) printf("      [diff] drv%u lba%lu off %zu got=%02x exp=%02x\n",
                              d, (unsigned long)mem_lba, i, got[i], ex[i]);
        return i == 65536;
    }
    // 通道帧账（拔盘冻结判据用 ✓）
    struct Fz { uint32_t ftx, frx, cqe, rxf; };
    Fz snap(unsigned d) {
        SasPairTop &p = arr->chan(d);
        return Fz{ p.hba.o_frames_tx.read().to_uint(), p.hba.o_frames_rx.read().to_uint(),
                   p.hba.o_cqes.read().to_uint(),     p.hdd.o_rx_frames.read().to_uint() };
    }
    bool frozen(unsigned d, const Fz &s) {
        Fz n = snap(d);
        return n.ftx == s.ftx && n.frx == s.frx && n.cqe == s.cqe && n.rxf == s.rxf;
    }

    void run() {
        printf("[TB] 预登记判据 A1..A8（RAID⇄HBA⇄link⇄HDD×4 交互 ✓）\n");
        while (!rst_n.read()) tick(1);
        tick(4);
        for (unsigned d = 0; d < 4; d++) arr->hba_setup(d);
        arr->raid.vol_config(0, RAID_LVL_R5, 4, 65536, 768);
        tick(4);

        vexp.assign((size_t)768 * 512, 0);
        std::vector<uint8_t> w((size_t)768 * 512); pat_fill(w.data(), w.size(), 0x11);

        // ── A1 全条带写 + 读回 ──
        {
            bool okw = arr->raid.vdisk_write(0, 0, w.data(), 768);
            std::memcpy(vexp.data(), w.data(), w.size());
            uint32_t io_wr[4];
            for (unsigned d = 0; d < 4; d++) io_wr[d] = arr->io_cnt[d];   // 写后快照 ✓
            std::vector<uint8_t> r((size_t)768 * 512, 0);
            bool okr = arr->raid.vdisk_read(0, 0, r.data(), 768);
            bool rdeq = (std::memcmp(r.data(), vexp.data(), vexp.size()) == 0);
            bool io2 = true;
            for (unsigned d = 0; d < 4; d++) if (io_wr[d] != 2u) io2 = false;
            if (!(okw && okr && rdeq && arr->raid.cnt_full() == 2 && io2))
                printf("      [A1 dbg] okw=%d okr=%d rdeq=%d cfull=%u io_wr=%u,%u,%u,%u\n",
                       okw, okr, rdeq, arr->raid.cnt_full(),
                       io_wr[0], io_wr[1], io_wr[2], io_wr[3]);
            chk(okw && okr && rdeq && arr->raid.cnt_full() == 2 && io2,
                "A1 基线：2 全条带写 ✓ 读回全对 ✓ c_full=2 ✓ 每通道恰 2 次成员 I/O ✓");
        }
        // ── A2 成员存储级对拍（含 TB 独立 XOR 现算的两行 P ✓）──
        {
            std::vector<uint8_t> p0, p1;
            xor3(&vexp[(size_t)0 * 512], &vexp[(size_t)128 * 512], &vexp[(size_t)256 * 512], p0);   // row0 P ✓
            xor3(&vexp[(size_t)384 * 512], &vexp[(size_t)512 * 512], &vexp[(size_t)(384 + 256) * 512], p1); // row1 P（k=0,1,2 ✓）
            bool ok = drv_unit_eq(0, 0,   &vexp[(size_t)0 * 512])     && drv_unit_eq(0, 128, &vexp[(size_t)384 * 512])
                   && drv_unit_eq(1, 0,   &vexp[(size_t)128 * 512])   && drv_unit_eq(1, 128, &vexp[(size_t)(384 + 128) * 512])
                   && drv_unit_eq(2, 0,   &vexp[(size_t)256 * 512])   && drv_unit_eq(2, 128, p1.data())
                   && drv_unit_eq(3, 0,   p0.data())                  && drv_unit_eq(3, 128, &vexp[(size_t)(384 + 256) * 512]);
            chk(ok, "A2 成员存储级：4 盘 × 2 单元 == 期望（含独立现算 P ✓ HDD 真存储 ✓）");
        }
        // ── A3 降级读（拔盘 1）+ 帧冻结 ──
        {
            Fz s1 = snap(1);
            arr->raid.member_fail(0, 1);
            std::vector<uint8_t> r((size_t)768 * 512, 0);
            bool okr = arr->raid.vdisk_read(0, 0, r.data(), 768);
            bool rdeq = (std::memcmp(r.data(), vexp.data(), vexp.size()) == 0);
            bool fz = frozen(1, s1);
            if (!(okr && rdeq && fz)) {
                Fz n = snap(1);
                printf("      [A3 dbg] okr=%d rdeq=%d fz=%d st=%u  ch1: ftx %u->%u frx %u->%u cqe %u->%u rxf %u->%u\n",
                       okr, rdeq, fz, arr->raid.vol[0].state, s1.ftx, n.ftx, s1.frx, n.frx,
                       s1.cqe, n.cqe, s1.rxf, n.rxf);
            }
            chk(okr && rdeq && arr->raid.vol[0].state == VOL_DEGRADED && fz,
                "A3 降级读：读全对 ✓ DEGRADED ✓ ch1 帧计数纹丝不动 ✓（不发帧给失败盘 ✓）");
        }
        // ── A4 降级写（D0 单元）──
        {
            std::vector<uint8_t> w2((size_t)20 * 512); pat_fill(w2.data(), w2.size(), 0x22);
            Fz s1 = snap(1);
            uint32_t r0 = arr->raid.cnt_rmw();
            bool okw = arr->raid.vdisk_write(0, 100, w2.data(), 20);
            std::memcpy(&vexp[(size_t)100 * 512], w2.data(), w2.size());
            std::vector<uint8_t> r((size_t)20 * 512, 0);
            bool okr = arr->raid.vdisk_read(0, 100, r.data(), 20);
            bool rdeq = (std::memcmp(r.data(), w2.data(), w2.size()) == 0);
            bool fz = frozen(1, s1);
            if (!(okw && okr && rdeq && (arr->raid.cnt_rmw() - r0 == 1) && fz))
                printf("      [A4 dbg] okw=%d okr=%d rdeq=%d rmw+%u fz=%d\n",
                       okw, okr, rdeq, arr->raid.cnt_rmw() - r0, fz);
            chk(okw && okr && rdeq && (arr->raid.cnt_rmw() - r0 == 1) && fz,
                "A4 降级写：读回 ✓ c_rmw+1 ✓ ch1 仍冻结 ✓（只动 D0 与 P ✓）");
        }
        // ── A5 换盘重建 ──
        {
            for (uint64_t l = 0; l < 256; l++)                       // drv1 两单元 wipe 成 0xAA ✓
                std::memset(arr->chan(1).hdd.lba_ptr(l), 0xAA, 512);
            uint32_t rb0 = arr->raid.cnt_rb();
            arr->raid.member_insert(0, 1);
            int guard = 0;
            while (arr->raid.vol[0].state == VOL_REBUILDING && guard++ < 8)
                arr->raid.rebuild_step(0);
            bool d1ok = drv_unit_eq(1, 0, &vexp[(size_t)128 * 512]) &&
                        drv_unit_eq(1, 128, &vexp[(size_t)(384 + 128) * 512]);
            std::vector<uint8_t> r((size_t)768 * 512, 0);
            bool okr = arr->raid.vdisk_read(0, 0, r.data(), 768);
            bool rdeq = (std::memcmp(r.data(), vexp.data(), vexp.size()) == 0);
            if (!(arr->raid.vol[0].state == VOL_ONLINE && (arr->raid.cnt_rb() - rb0 == 256) &&
                  d1ok && okr && rdeq))
                printf("      [A5 dbg] st=%u rb+%u d1ok=%d okr=%d rdeq=%d guard=%d\n",
                       arr->raid.vol[0].state, arr->raid.cnt_rb() - rb0, d1ok, okr, rdeq, guard);
            chk(arr->raid.vol[0].state == VOL_ONLINE && (arr->raid.cnt_rb() - rb0 == 256) &&
                d1ok && okr && rdeq,
                "A5 换盘重建：ONLINE ✓ c_rb+256 ✓ drv1 存储 == 期望片 ✓ 读全卷 ✓（帧级重建 ✓）");
        }
        // ── A6 账目闭合 ──
        {
            bool ok = true;
            for (unsigned d = 0; d < 4; d++) {
                SasPairTop &p = arr->chan(d);
                bool a = (p.hba.o_cqes.read().to_uint() == arr->io_cnt[d]);
                bool b = (p.hdd.o_cmds_good.read().to_uint() == arr->io_cnt[d]);
                bool c = (p.hba.o_cmds_err.read().to_uint() == 0u);
                bool e = (p.hba.o_tag_mismatch.read().to_uint() == 0u);
                bool f = (p.lk_dropped.read().to_uint() == 0u);
                if (!(a && b && c && e && f))
                    printf("      [A6 dbg] ch%u cqes=%u io=%u good=%u err=%u tagmm=%u drop=%u\n",
                           d, p.hba.o_cqes.read().to_uint(), arr->io_cnt[d],
                           p.hdd.o_cmds_good.read().to_uint(), p.hba.o_cmds_err.read().to_uint(),
                           p.hba.o_tag_mismatch.read().to_uint(), p.lk_dropped.read().to_uint());
                ok &= a && b && c && e && f;
            }
            chk(ok, "A6 账目闭合：o_cqes==io_cnt==cmds_good ✓ 零错/零失配/零丢帧 ✓（4 通道 ✓）");
        }
        // ── A7 块账 ──
        {
            bool ok = true;
            for (unsigned d = 0; d < 4; d++) {
                SasPairTop &p = arr->chan(d);
                ok &= (p.hdd.o_lba_read.read().to_uint()  == (uint32_t)arr->blk_rd[d]);
                ok &= (p.hdd.o_lba_write.read().to_uint() == (uint32_t)arr->blk_wr[d]);
            }
            chk(ok, "A7 块账：hdd.o_lba_read/write == 逐盘账（链路往返块数逐步对账 ✓）");
        }
        // ── A8 负控：两缺 ⇒ OFFLINE ⇒ 读写皆拒 + 全通道帧冻结 ──
        {
            arr->raid.member_fail(0, 2);
            arr->raid.member_fail(0, 3);
            Fz s[4] = { snap(0), snap(1), snap(2), snap(3) };
            std::vector<uint8_t> x(512, 0);
            bool rej = !arr->raid.vdisk_read(0, 0, x.data(), 1) &&
                       !arr->raid.vdisk_write(0, 0, x.data(), 1);
            bool frz = frozen(0, s[0]) && frozen(1, s[1]) && frozen(2, s[2]) && frozen(3, s[3]);
            chk(arr->raid.vol[0].state == VOL_OFFLINE && rej && frz,
                "A8 负控：两缺 ⇒ OFFLINE ✓ 读写皆拒 ✓ 四通道帧全冻结 ✓（拒绝在 RAID 层 ✓）");
        }
        printf("[合计] PASS=%d FAIL=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

int sc_main(int argc, char **argv) {
    (void)argc; (void)argv;
    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;
    SasRaidArrayTop arr("arr");
    TbArr tb("tb");
    DummyAmbaMaster st0("st0"), st1("st1"), st2("st2"), st3("st3");
    tb.clk(clk); tb.rst_n(rst_n); tb.arr = &arr;
    arr.clk(clk); arr.rst_n(rst_n);
    st0.m.bind(arr.ch0.hba.reg_s);
    st1.m.bind(arr.ch1.hba.reg_s);
    st2.m.bind(arr.ch2.hba.reg_s);
    st3.m.bind(arr.ch3.hba.reg_s);
    rst_n.write(false);
    sc_core::sc_start(200, sc_core::SC_NS);            // 20 拍复位 ✓
    rst_n.write(true);
    sc_core::sc_start();                                // 到 sc_stop ✓
    int rc = (tb.fail_cnt == 0) ? 0 : 1;
    printf("%s\n", rc == 0 ? "TB_RAID_ARRAY PASS" : "TB_RAID_ARRAY FAIL");
    return rc;
}
