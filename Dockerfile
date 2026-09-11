FROM debian:jessie-slim

# Point APT to the permanent Debian Archive
RUN echo "deb http://archive.debian.org/debian/ jessie main contrib non-free" > /etc/apt/sources.list && \
    echo "deb http://archive.debian.org/debian-security/ jessie/updates main contrib non-free" >> /etc/apt/sources.list

# Disable validity check for archived repositories
RUN echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99no-check-valid-until && \
    echo 'APT::Get::AllowUnauthenticated "true";' >> /etc/apt/apt.conf.d/99no-check-valid-until

ENV DEBIAN_FRONTEND=noninteractive

# Enable 32-bit architecture and install era-appropriate build tools
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --force-yes \
    build-essential \
    gcc \
    g++ \
    make \
    libncurses5-dev \
    libncurses5:i386 \
    libstdc++6:i386 \
    zlib1g:i386 \
    libc6:i386 \
    bison \
    flex \
    git \
    gnupg \
    gperf \
    zip \
    curl \
    zlib1g-dev \
    u-boot-tools \
    perl \
    rsync \
    bc \
    python \
    wget \
    cpio \
    && rm -rf /var/lib/apt/lists/*

# Map non-root user credentials matching host system
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} builder && \
    useradd -u ${USER_ID} -g builder -m -s /bin/bash builder

USER builder
WORKDIR /workspace