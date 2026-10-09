# sas_raid — SAS RAID SystemC TLM model

硬件 RAID 加速器的事务级模型（SystemC 2.3.4 / C++14）：对 Host 呈现**单个虚拟卷**，
命令条带化到成员盘；RAID 数学（XOR / GF(2⁸)）由"硬件引擎"完成，**无固件 CPU**。

- 等级：RAID 0 / 1 / 5（RMW + 全条带）/ 6（P + Q）/ 10；条带 64K–1M
- 地址翻译：与规格五式逐位一致（left-asymmetric 旋转）；条带映射/卷表/写洞日志/重建扫描
- 降级路径：R5 单缺重建读；R6 **单缺 + 双缺 2×2 GF 解**；换盘重建（逐盘分轮、checkpoint）
- 成员后端**可绑**：内存阵列（单元台）或**真帧级全栈**（RAID ⇄ SAS HBA TLM ⇄ link ⇄ HDD TLM，联合台）
- 判据：**7 台 / 48 条全绿**（含联合交互台 8 条：条带写→读回→拔盘→降级→换盘重建→数据全对）

## 目录

```
model/raid/   sas_raid_tlm.{h,cpp}      RAID 层（卷表/条带/等级/GF/重建/日志 ✓）
model/top/    sas_raid_array_top.h      联合顶层（RAID ⇄ 4×(HBA⇄link⇄HDD) ✓）
model/tb/     tb_raid_*.cpp + 公共件     7 个判据台（各自 sc_main + 终判行 ✓）
sw/           sas_raid_regs.h           唯一基准：几何/五式/卷表/日志/寄存器
docs/         design / verification / errata_and_crosscheck
```

## 构建与回归

```sh
cd model
make            # 构建全部台（tb_raid_array 需兄弟仓，见下）
sh regress.sh   # ⇒ 7 台 ALL PASS ✓（只认终判行 ✓）
```

`tb_raid_array`（M6 联合交互台）依赖两个**公开兄弟仓**：

```sh
git clone https://github.com/tlmer/sas_hba
git clone https://github.com/tlmer/sas_hdd
# 与本仓并列放置（目录名 sas_hba / sas_hdd ✓），无 AMBA-PV 时可用
#   make HBA_DIR=<abs>/sas_hba/tlm HDD_DIR=<abs>/sas_hdd/tlm
```

## 接口一览

```cpp
SasRaidTlm raid("raid");                 // + clk / rst_n
raid.vol_config(0, RAID_LVL_R5, 4, 65536, /*cap_blocks*/768);
raid.backend_read  = ...;                // 成员后端：内存 或 帧级全栈（函数指针 ✓）
raid.backend_write = ...;
raid.vdisk_read (0, lba, buf, nblk);     // 虚拟卷读写
raid.vdisk_write(0, lba, buf, nblk);
raid.member_fail(0, 1);                  // 故障注入：拔盘 ⇒ DEGRADED
raid.member_insert(0, 1);                // 插回 ⇒ REBUILDING
while (raid.vol[0].state == VOL_REBUILDING) raid.rebuild_step(0);   // 重建（checkpoint ✓）
raid.journal_count();  raid.journal_replay(0);                      // 写洞日志 ✓
```

## 备注

- 帧/命令口径与 `sas_hba` / `sas_hdd` TLM 逐字节一致（CDB@36 / tag@16-17 / LUN@24-31 ✓）
- 元命令（INQUIRY / READ CAPACITY / MODE SENSE / REQUEST SENSE）由本层本地应答，
  数据与 HDD 侧字节级一致；容量 = 卷容量
- 本文档与 `docs/` 由源码注释同源生成维护
