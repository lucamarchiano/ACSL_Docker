#======================================
# Stage 1: Setup and install packages
#======================================
FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && sudo apt-get upgrade && \
    apt-get install -y \
    # Common
    git nano libasio-dev usbutils linux-firmware dkms software-properties-common lsb-release build-essential make g++ gcc locales tzdata\
    # CMake prerequisite 
    ca-certificates gpg wget \
    # Access Point setup prerequisites
    libgtk-3-dev pkg-config hostapd libqrencode-dev libpng-dev iw iptables dnsmasq \
    # Boost 1.86 prerequisites
    python3-dev autotools-dev libicu-dev libbz2-dev \
    # VIO prerequirites
    libssl-dev libusb-1.0-0-dev libudev-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

#==========================================
# Stage 2: Install CMake 3.26 from source
#==========================================
FROM base AS cmake-builder

RUN wget https://apt.kitware.com/keys/kitware-archive-latest.asc && \
    gpg --dearmor kitware-archive-latest.asc && \
    mv kitware-archive-latest.asc.gpg /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main" > /etc/apt/sources.list.d/kitware.list && \
    apt-get update && \
    apt-get install -y cmake=3.26.* cmake-data=3.26.* && \
    echo -e "Package: cmake\nPin: version 3.26*\nPin-Priority: 1001" > /etc/apt/preferences.d/cmake-pin && \
    echo -e "Package: cmake-data\nPin: version 3.26*\nPin-Priority: 1001" > /etc/apt/preferences.d/cmake-data-pin && \
    rm -rf /var/lib/apt/lists/*

#=========================================
# Stage 3: Install Micro-XRCE-DDS-Agent
#=========================================
FROM base AS uxrce-builder

# Copy CMake binaries and data from cmake-builder
COPY --from=cmake-builder /usr/bin/cmake /usr/bin/cmake
COPY --from=cmake-builder /usr/share/cmake-3.26 /usr/share/cmake-3.26

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

#==========================================
# Stage 4: Install Boost 1.86 from source
#==========================================
FROM base AS boost-builder

RUN git clone --recursive --branch boost-1.86.0 https://github.com/boostorg/boost.git /boost && \
    cd /boost && \
    ./bootstrap.sh --prefix=/usr/local && \
    ./b2 install -j"$(nproc)" && \
    rm -rf /boost


#==============================
# Stage 5: Setup VIO packages
#==============================
FROM base AS vio-builder

# Copy CMake binaries and data from cmake-builder
COPY --from=cmake-builder /usr/bin/cmake /usr/bin/cmake
COPY --from=cmake-builder /usr/share/cmake-3.26 /usr/share/cmake-3.26


RUN git clone --branch v2.53.1 --recurse-submodules https://github.com/IntelRealSense/librealsense.git

WORKDIR /librealsense

RUN mkdir -p /etc/udev/rules.d/ && \
    ./scripts/setup_udev_rules.sh && \
    mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=TRUE && \
    make uninstall && make clean && \ 
    make -j$(nproc) && \
    make install


#=======================
# Stage 6: Final build
#=======================
FROM base 

# Copy CMake binaries and data from cmake-builder
COPY --from=cmake-builder /usr/bin/cmake /usr/bin/cmake
COPY --from=cmake-builder /usr/share/cmake-3.26 /usr/share/cmake-3.26

# Copy Micro-XRCE-DDS-Agent runtime and libraries
COPY --from=uxrce-builder /usr/local/bin /usr/local/bin
COPY --from=uxrce-builder /usr/local/lib /usr/local/lib
COPY --from=uxrce-builder /usr/local/include /usr/local/include

# Copy Boost libraries from boost-builder stage
COPY --from=boost-builder /usr/local/include /usr/local/include
COPY --from=boost-builder /usr/local/lib /usr/local/lib

# Copy librealsense libraries and headers
COPY --from=vio-builder /usr/local/lib /usr/local/lib
COPY --from=vio-builder /usr/local/include /usr/local/include

RUN ldconfig && \
    useradd -ms /bin/bash user && \
    echo "user:user" | chpasswd && \
    usermod -aG sudo user && \
    echo "user ALL=(ALL) PASSWD:ALL" >> /etc/sudoers && \
    echo "export PS1='\[\033[1;33m\]\u@\h\[\033[0m\]:\[\033[1;34m\]\w\[\033[0m\]\$ '" >> /home/user/.bashrc && \    
    echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc

USER user
WORKDIR /home/user

CMD ["bash", "-l"]
LABEL org.opencontainers.image.source="https://github.com/lucamarchiano/ACSL_Docker"

# TODO: final build, add RTC support, add AP, add UART setups