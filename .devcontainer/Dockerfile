# Use the official ARM64 compatible development image
FROM nvidia/cuda:12.6.3-devel-ubuntu22.04

# Set non-interactive environment for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update system and install basic utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

