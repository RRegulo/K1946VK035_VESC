FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    python3 \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    gdb-multiarch \
    wget \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# OpenOCD с поддержкой K1921VK035 (в Ubuntu-пакете его нет)
RUN wget -qO /tmp/openocd.tar.gz \
        "https://github.com/DCVostok/openocd-k1921vk/releases/download/v0.12.0-k1921vk/xpack-openocd-k1921vk-0.12.0-k1921vk-linux-x64.tar.gz" && \
    mkdir -p /tools/openocd-k1921vk && \
    tar -xzf /tmp/openocd.tar.gz --strip-components=1 -C /tools/openocd-k1921vk && \
    rm /tmp/openocd.tar.gz && \
    ln -s /tools/openocd-k1921vk/bin/openocd /usr/local/bin/openocd-k1921vk

# В архиве отсутствует target-конфигурация К1921ВК035.
COPY .vscode/k1921vk035.cfg /tools/openocd-k1921vk/openocd/scripts/target/k1921vk035.cfg
