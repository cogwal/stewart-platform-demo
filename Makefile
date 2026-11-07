# Run these from the host (Mac or Ubuntu notebook). Everything else happens in the dev container.
CAN_IFACE ?= vcan0
export CAN_IFACE
COMPOSE   := docker compose
RUN       := $(COMPOSE) run --rm dev

.PHONY: help vcan image shell test gen ctest spmaster contract sim sim-leg s0-check clean

help:
	@echo "make vcan      - Mac: bring up $(CAN_IFACE) in the Colima VM"
	@echo "make image     - build the dev image"
	@echo "make shell     - interactive shell in the dev container"
	@echo "make test      - all tests (contract, emulator, C master; bus tests need $(CAN_IFACE))"
	@echo "make gen       - regenerate master/generated/ from the contract and config/platform.yaml"
	@echo "make ctest     - C master unit tests only (fake bus, fake time)"
	@echo "make spmaster  - build the C master on CANopenLinux (build/master/spmaster)"
	@echo "make contract  - print the startup SDO sequence for drive node 1"
	@echo "make sim       - Stewart simulation: 6 emulated drives + C master + 3D web view on :8080"
	@echo "make sim-leg   - single leg: 1 emulated drive + Python test master + web view on :8080"
	@echo "make s0-check  - build the CANopenLinux demo, run it on $(CAN_IFACE), check it with python-canopen"
	@echo "make clean     - remove build output"

vcan:
	scripts/mac/colima-vcan.sh $(CAN_IFACE)

image:
	$(COMPOSE) build dev

shell:
	$(RUN) bash

test:
	$(RUN) pytest

gen:
	$(RUN) python -m contract.gen_c

ctest:
	$(RUN) make -C master test

spmaster:
	$(RUN) make -C master spmaster

contract:
	$(RUN) python -m contract 1

sim:
	$(RUN) tools/run_sim.sh $(CAN_IFACE)

sim-leg:
	$(RUN) env SIM_LEGS=1 SIM_MASTER=py tools/run_sim.sh $(CAN_IFACE)

s0-check:
	$(RUN) tools/s0_demo_check.sh $(CAN_IFACE)

clean:
	rm -rf build
