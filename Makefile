# Run these from the host (Mac or Ubuntu notebook). Everything else happens in the dev container.
CAN_IFACE ?= vcan0
export CAN_IFACE
COMPOSE   := docker compose
RUN       := $(COMPOSE) run --rm dev

.PHONY: help vcan image shell s0-check clean

help:
	@echo "make vcan      - Mac: bring up $(CAN_IFACE) in the Colima VM"
	@echo "make image     - build the dev image"
	@echo "make shell     - interactive shell in the dev container"
	@echo "make s0-check  - build the CANopenLinux demo, run it on $(CAN_IFACE), check it with python-canopen"
	@echo "make clean     - remove build output"

vcan:
	scripts/mac/colima-vcan.sh $(CAN_IFACE)

image:
	$(COMPOSE) build dev

shell:
	$(RUN) bash

s0-check:
	$(RUN) tools/s0_demo_check.sh $(CAN_IFACE)

clean:
	rm -rf build
