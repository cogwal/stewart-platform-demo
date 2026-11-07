# C master: portable core (core/), generated config (generated/), CANopenNode adapter (canopennode/),
# Linux app on CANopenLinux (linux/), unit tests (tests/). Run from the repo root: make -C master <target>
# (the top-level Makefile wraps these in the dev container).

BUILD   ?= ../build/master
COL     := ../third_party/CANopenLinux
CON     := $(COL)/CANopenNode
CC      ?= gcc
WARN    := -Wall -Wextra -Wshadow -Wno-unused-parameter
CORE_SRC := core/sp_core.c core/sp_kin.c core/sp_motion.c core/sp_app.c generated/sp_config_gen.c
CORE_INC := -Icore -Igenerated

.PHONY: all test spmaster clean

all: test spmaster

# ---- unit tests (no CAN) ---------------------------------------------------------------------
$(BUILD)/test_core: $(CORE_SRC) tests/test_core.c tests/fake_bus.c tests/fake_bus.h core/*.h generated/*.h
	@mkdir -p $(BUILD)
	$(CC) -std=c99 -O1 -g $(WARN) -Werror -fsanitize=address,undefined $(CORE_INC) -Itests \
	    $(CORE_SRC) tests/test_core.c tests/fake_bus.c -lm -o $@

$(BUILD)/kin_dump: $(CORE_SRC) tests/kin_dump.c core/*.h generated/*.h
	@mkdir -p $(BUILD)
	$(CC) -std=c99 -O1 -g $(WARN) -Werror $(CORE_INC) $(CORE_SRC) tests/kin_dump.c -lm -o $@

test: $(BUILD)/test_core $(BUILD)/kin_dump
	$(BUILD)/test_core

# ---- spmaster: the master on CANopenLinux ----------------------------------------------------
CO_SRC := \
	$(COL)/CO_driver.c $(COL)/CO_error.c $(COL)/CO_epoll_interface.c $(COL)/CO_storageLinux.c \
	$(COL)/CO_main_basic.c \
	$(CON)/301/CO_ODinterface.c $(CON)/301/CO_NMT_Heartbeat.c $(CON)/301/CO_HBconsumer.c \
	$(CON)/301/CO_Emergency.c $(CON)/301/CO_SDOserver.c $(CON)/301/CO_SDOclient.c $(CON)/301/CO_TIME.c \
	$(CON)/301/CO_SYNC.c $(CON)/301/CO_PDO.c $(CON)/301/crc16-ccitt.c $(CON)/301/CO_fifo.c \
	$(CON)/303/CO_LEDs.c $(CON)/304/CO_GFC.c $(CON)/304/CO_SRDO.c $(CON)/305/CO_LSSslave.c \
	$(CON)/305/CO_LSSmaster.c $(CON)/309/CO_gateway_ascii.c $(CON)/storage/CO_storage.c \
	$(CON)/extra/CO_trace.c $(CON)/CANopen.c
APP_SRC := generated/OD.c canopennode/sp_co_adapter.c linux/sp_linux_app.c
# Our OD.h must win over the demo OD in CANopenNode/example: generated/ comes first.
CO_DEFS := -DCO_SINGLE_THREAD -DCO_USE_APPLICATION -DMAIN_THREAD_INTERVAL_US=1000 \
	-DNMT_CONTROL=CO_NMT_STARTUP_TO_OPERATIONAL -DGATEWAY_ENABLE=false \
	'-DCO_CONFIG_HB_CONS=(CO_CONFIG_HB_CONS_ENABLE|CO_CONFIG_HB_CONS_CALLBACK_MULTI|CO_CONFIG_HB_CONS_QUERY_FUNCT|CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE|CO_CONFIG_GLOBAL_FLAG_TIMERNEXT|CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)'
CO_INC := -Igenerated -Icore -Icanopennode -I$(COL) -I$(CON)

$(BUILD)/spmaster: $(CORE_SRC) $(APP_SRC) $(CO_SRC) core/*.h generated/*.h canopennode/*.h
	@mkdir -p $(BUILD)
	$(CC) -O2 -g -Wall $(CO_DEFS) $(CO_INC) $(CORE_SRC) $(APP_SRC) $(CO_SRC) -lm -o $@

spmaster: $(BUILD)/spmaster

clean:
	rm -rf $(BUILD)
