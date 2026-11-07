# Run these from the host (Mac or Ubuntu notebook). Everything else happens in the dev container.
CAN_IFACE ?= vcan0
export CAN_IFACE
COMPOSE   := docker compose
RUN       := $(COMPOSE) run --rm dev

.PHONY: help vcan image shell test contract s0-check clean

help:
	@echo "make vcan      - Mac: bring up $(CAN_IFACE) in the Colima VM"
	@echo "make image     - build the dev image"
	@echo "make shell     - interactive shell in the dev container"
	@echo "make test      - run the tests that need no CAN bus (contract checks)"
	@echo "make contract  - print the startup SDO sequence for drive node 1"
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

contract:
	$(RUN) python -m contract 1

s0-check:
	$(RUN) tools/s0_demo_check.sh $(CAN_IFACE)

clean:
	rm -rf build
