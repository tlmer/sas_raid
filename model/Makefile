#=============================================================================
# raid/tlm/model/Makefile — RAID TLM 构建  [2026-10-09 建]
#   口径照全工程 ✓：SYSTEMC 可用环境变量覆盖 ✓；TB = 各自 sc_main + 终判行 ✓
#   ⚠ 新增台 ⇒ 只改 TBS 一行 ✓（regress.sh 自动同步 ✓）
#   ★ tb_raid_array（M6 联合交互台 ✓）需 **兄弟仓 sas_hba / sas_hdd**（并列 clone ✓）
#     与 AMBA-PV 头 ⇒ 专用规则 ✓（未 clone 兄弟仓时可只跑前 6 台 ✓）
#=============================================================================
SYSTEMC      ?= /opt/tools/systemc2.34
AMBA_PV_HOME ?= /mnt/hdd1/vdk2024.03/SLS/linux/IP_common/AMBA_TLM2_BL/AMBA-PV
HBA_DIR      ?= ../../sas_hba
HDD_DIR      ?= ../../sas_hdd
CXX     ?= g++
CXXFLAGS ?= -std=c++14 -O1 -Wall -I$(SYSTEMC)/include -Iraid -Itb -I../sw
LDFLAGS  ?= -L$(SYSTEMC)/lib-linux64 -lsystemc -Wl,-rpath,$(SYSTEMC)/lib-linux64

TBS = tb_raid_map tb_raid_r01 tb_raid_r5 tb_raid_r6 tb_raid_rebuild tb_raid_journal tb_raid_array

all: $(TBS)

tb_%: tb/tb_%.cpp raid/sas_raid_tlm.cpp raid/sas_raid_tlm.h ../sw/sas_raid_regs.h
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

# ── M6 联合台（显式规则优先于上面的模式规则 ✓）：RAID + HBA + link + HDD 四源 ✓ ──
ARRAY_FLAGS = $(CXXFLAGS) -I$(AMBA_PV_HOME)/include -Itop \
              -I$(HBA_DIR)/model/hba -I$(HBA_DIR)/model/link -I$(HBA_DIR)/model/top \
              -I$(HBA_DIR)/model/tb -I$(HBA_DIR)/sw \
              -I$(HDD_DIR)/model/hdd -I$(HDD_DIR)/sw
ARRAY_SRC   = tb/tb_raid_array.cpp raid/sas_raid_tlm.cpp \
              $(HBA_DIR)/model/hba/sas_hba_tlm.cpp \
              $(HBA_DIR)/model/link/sas_link_tlm.cpp \
              $(HDD_DIR)/model/hdd/sas_hdd_tlm.cpp
tb_raid_array: $(ARRAY_SRC) top/sas_raid_array_top.h raid/sas_raid_tlm.h ../sw/sas_raid_regs.h \
               $(HBA_DIR)/model/top/sas_pair_top.h $(HBA_DIR)/model/hba/sas_hba_tlm.h \
               $(HBA_DIR)/model/link/sas_link_tlm.h $(HDD_DIR)/model/hdd/sas_hdd_tlm.h
	$(CXX) $(ARRAY_FLAGS) $(ARRAY_SRC) $(LDFLAGS) -o $@

clean:
	rm -f $(TBS)

.PHONY: all clean
