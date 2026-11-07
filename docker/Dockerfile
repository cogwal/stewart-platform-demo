# Dev image: CANopenLinux build tools, can-utils and the Python simulator stack.
# CAN interfaces come from the host kernel (vcan0 in the Colima VM on the Mac, vcan0/can0 on
# the Ubuntu notebook); run the container with host networking to see them.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        can-utils \
        git \
        iproute2 \
        make \
        python3 \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt
ENV PATH="/opt/venv/bin:${PATH}" \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /workspace
