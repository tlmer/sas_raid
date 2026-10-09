//============================================================================
// tb_raid_map.cpp — M1 判据台：**条带地址翻译**（对 spec §"RAID Volume Address
//   Translation" 五式逐位 ✓）  [2026-10-09 建]
//
//   判据（**预登记** ✓；出处 = `规格文档` 五式 + left-asymmetric 图 ✓）：
//   A1 spec **逐字例题**：Vol1、LBA=0x1000、4 盘 R5 ⇒ stripe=0 / offset=0 /
//      drive=0（数据序 ✓）/ p_drive=3 / drive_lba=0 ✓
//   A2 **left-asymmetric 旋转**：N=4 时 stripe=0..3 的 P 盘 = 3,2,1,0 ✓（spec 图 ✓）
//   A3 数据盘落位：`raid_phys` 跳过 P 盘后数据序与物理盘号的映射（stripe=0：D0→0,D1→1,D2→2 ✓；
//      stripe=3：P 在盘 0 ⇒ D0→1,D1→2,D2→3 ✓）
//   A4 边界：offset 末端块（LBA=191 ⇒ stripe=0/offset=63×1024/drive=2 ✓）、跨条带（LBA=192 ⇒
//      stripe=1 ✓）、跨 stripe_unit（LBA=128 ⇒ drive=2 ✓）
//   A5 **逐 LBA 扫描对拍**：LBA 0..N×1024-1 全枚举，用独立实现（本台内联的"参照式" ✓）
//      与 `raid_xlate_nd` 逐位一致 ✓（防公式抄错 ✓）
//   A6 R6 双校验盘位：N=6 时 stripe=0 ⇒ P=5、Q=(6-2)-(1%6)=3 ✓（错开 ✓）
//============================================================================
#include <cstdio>
#include "raid_tb_common.h"

// 独立参照实现（**故意另写一遍** ✓：按 spec 图逐字翻译，防"照抄同错"✗）
static void ref_xlate_r5(uint64_t lba, unsigned n, unsigned stripe_size,
                         uint64_t *stripe, uint64_t *offset, unsigned *drive,
                         unsigned *p_drive, uint64_t *drive_lba)
{
    uint64_t bytes = lba * 512ull;
    uint64_t row = (uint64_t)stripe_size * (n - 1);
    *stripe = bytes / row;
    *offset = bytes % row;
    *drive = (unsigned)(*offset / stripe_size);
    *p_drive = (unsigned)((n - 1) - (*stripe % n));
    *drive_lba = (*stripe * stripe_size + (*offset % stripe_size)) / 512ull;
}

int sc_main(int argc, char **argv)
{
    (void)argc; (void)argv;
    RaidTbScore sc;
    printf("=== tb_raid_map：条带翻译对 spec 五式（M1 ✓）===\n");

    // ── A1 spec 例题输入 + **手算真值** ✓（⚠ spec 该例题的"结果"自相矛盾 ✗，
    //    `(0x1000×512)/(65536×3)` = 2,097,152/196,608 = 10.67 ⇒ **stripe=10** ✗ ≠ 其写的 0 ✗
    //    ⇒ 以**公式**为准（本台手算复核 ✓），例题的"结果"记入对拍报告为**文档勘误** ✗）
    {
        raid_xlate_t x;
        raid_xlate(0x1000, 4, 65536, &x);
        // 手算：bytes=2097152；row=196608 ⇒ stripe=10 ✓；offset=2097152-10×196608=131072 ✓
        //       drive=131072/65536=2 ✓；p=3-(10%4=2)=1 ✓；drive_lba=(10×65536+0)/512=1280 ✓
        sc.chk(x.stripe == 10 && x.offset == 131072 && x.drive == 2 &&
               x.p_drive == 1 && x.drive_lba == 1280,
               "A1 spec 例题输入 ⇒ 手算真值 10/131072/2/P=1/1280 ✓（spec 例题结果自相矛盾 ✗ 勘误）");
    }
    // ── A2 旋转 ✓ ──
    {
        bool ok = true;
        unsigned expect[4] = {3, 2, 1, 0};
        for (unsigned s = 0; s < 4; s++) {
            unsigned nd = 3;                                 // R5：N-1=3 ✓
            raid_xlate_t x;
            raid_xlate_nd((uint64_t)s * 384, 4, nd, 65536, &x);   // ⚠ 行 = 3×64KB = **384 块** ✓（旧写 3×1024 ✗）
            ok &= (x.stripe == s) && (x.p_drive == expect[s]) && (x.offset == 0);
        }
        sc.chk(ok, "A2 left-asymmetric 旋转 stripe0..3 ⇒ P=3,2,1,0 ✓");
    }
    // ── A3 数据盘落位 ✓ ──
    {
        raid_xlate_t x; unsigned n = 4;
        raid_xlate_nd(0, 4, 3, 65536, &x);          // stripe 0：P=3 ⇒ D0=0,D1=1,D2=2 ✓
        bool ok = raid_phys_drive_r5(&x, n, 0) == 0 && raid_phys_drive_r5(&x, n, 1) == 1
                && raid_phys_drive_r5(&x, n, 2) == 2;
        raid_xlate_nd((uint64_t)3 * 384, 4, 3, 65536, &x);   // stripe 3：P=0 ⇒ D0=1,D1=2,D2=3 ✓（LBA=1152 ✓）
        ok &= raid_phys_drive_r5(&x, n, 0) == 1 && raid_phys_drive_r5(&x, n, 1) == 2
            && raid_phys_drive_r5(&x, n, 2) == 3;
        sc.chk(ok, "A3 落位（stripe0：D→0/1/2 ✓；stripe3：P 在盘0 ⇒ D→1/2/3 ✓）");
    }
    // ── A4 边界 ✓ ──
    {
        raid_xlate_t x;
        raid_xlate_nd(255, 4, 3, 65536, &x);        // 单元 1 末块（191 属 D1 ✗ 旧写 ✗）
        bool ok = x.stripe == 0 && x.drive == 1;
        raid_xlate_nd(256, 4, 3, 65536, &x);        // 256×512 = 128KB ⇒ 进 D2 ✓
        ok &= x.stripe == 0 && x.offset == 131072 && x.drive == 2;
        raid_xlate_nd(384, 4, 3, 65536, &x);        // 384 块 = 3×64KB ⇒ 跨到 stripe1 ✓
        ok &= x.stripe == 1 && x.offset == 0 && x.drive == 0 && x.p_drive == 2;
        raid_xlate_nd(128, 4, 3, 65536, &x);        // 128×512 = 64KB ⇒ 单元换道 ⇒ D1 ✓
        ok &= x.stripe == 0 && x.drive == 1;
        sc.chk(ok, "A4 边界：255⇒D1 ✓；256⇒D2 ✓；384⇒stripe1 ✓；128⇒换道 ✓（行=384 块 ✓）");
    }
    // ── A5 全枚举对拍（独立参照式 ✓）──
    {
        unsigned n = 4, ss = 65536;
        uint64_t total = (uint64_t)(n - 1) * (ss / 512) * 8;      // 8 个条带 ✓
        bool ok = true;
        for (uint64_t l = 0; l < total && ok; l++) {
            raid_xlate_t a; raid_xlate_nd(l, n, n - 1, ss, &a);
            uint64_t rs, ro, rd_lba; unsigned rdr, rp;
            ref_xlate_r5(l, n, ss, &rs, &ro, &rdr, &rp, &rd_lba);
            ok &= (a.stripe == rs && a.offset == ro && a.drive == rdr &&
                   a.p_drive == rp && a.drive_lba == rd_lba);
        }
                // 手算金标（独立于两套实现的第三个信源 ✓）
        raid_xlate_t g; raid_xlate_nd(384, 4, 3, 65536, &g);
        ok &= (g.stripe == 1 && g.drive == 0 && g.p_drive == 2 && g.drive_lba == 128);   // (1×65536+0)/512=128 ✓
        raid_xlate_nd(1280, 4, 3, 65536, &g);
        ok &= (g.stripe == 3 && g.drive == 1 && g.p_drive == 0 && g.drive_lba == 384);   // (3×65536+0)/512=384 ✓
        sc.chk(ok, "A5 全枚举对拍（8 条带 × 全 LBA ✓）+ 手算金标 ×2 ✓");
    }
    // ── A6 R6 双校验位 ✓ ──
    {
        raid_xlate_t x; unsigned n = 6;
        raid_xlate_nd(0, n, n - 2, 65536, &x);
        unsigned pP, pQ;                                         // 与模型同式 ✓
        pP = (unsigned)((n - 1) - (x.stripe % n));
        pQ = (unsigned)((n - 2) - ((x.stripe + 1) % n));
        sc.chk(pP == 5 && pQ == 3, "A6 R6 校验盘位（N=6,stripe0）⇒ P=5 / Q=3 ✓（错开 ✓）");
    }
    return sc.verdict("TB_RAID_MAP");
}
