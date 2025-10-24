# Stage 1: Base with ROS and preliminary packages
FROM osrf/ros:humble-desktop AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN ln -sf "$(which python3)" /usr/local/bin/python && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      swig \
      libirrlicht-dev \
      libgl1-mesa-glx \
      x11-xserver-utils \
      x11-apps \
      python3-pip \
      libbz2-dev \
      wget \
      nano && \
    rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir \
      numpy==1.24.0 \
      matplotlib \
      pytz \
      scipy

# Ensures all following RUN commands will be executed in a bash shell and not sh
SHELL ["/bin/bash", "-c"]


# Stage 2: Install CMake 3.26 from Kitware APT repo
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


# Stage 3: Build Boost 1.86 from source
FROM base AS boost-builder

RUN git clone --recursive --branch boost-1.86.0 https://github.com/boostorg/boost.git /boost && \
    cd /boost && \
    ./bootstrap.sh --prefix=/usr/local && \
    ./b2 install -j"$(nproc)" && \
    ldconfig && \
    rm -rf /boost

RUN ldconfig


# Stage 4: Build chrono_ros_interfaces package
FROM base AS chrono-ros-builder

WORKDIR /root/chrono_ros_interfaces_build
RUN mkdir -p src && \
    git clone https://github.com/projectchrono/chrono_ros_interfaces src/chrono_ros_interfaces && \
    source /opt/ros/humble/setup.bash && \
    colcon build


# Stage 5: Build PyChrono (Chrono 9.0.1) from source, sourcing chrono_ros_interfaces first
FROM base AS chrono-builder

# Copy CMake binaries and data from cmake-builder
COPY --from=cmake-builder /usr/bin/cmake /usr/bin/cmake
COPY --from=cmake-builder /usr/share/cmake-3.26 /usr/share/cmake-3.26

# Copy Boost installation from boost-builder (includes headers (/include) and libs (/lib))
COPY --from=boost-builder /usr/local/include /usr/local/include
COPY --from=boost-builder /usr/local/lib /usr/local/lib

# Copy chrono_ros_interfaces install space for sourcing setup
COPY --from=chrono-ros-builder /root/chrono_ros_interfaces_build/ /root/chrono_ros_interfaces_build/

RUN ldconfig

WORKDIR /root/chrono_build

# Ensure Python modules from build folder are discoverable
ENV PYTHONPATH=/root/chrono_build/build/bin:$PYTHONPATH

RUN git clone https://github.com/projectchrono/chrono . && \
    mkdir -p build && \
    cd build && \
    source /opt/ros/humble/setup.bash && \
    source /root/chrono_ros_interfaces_build/install/local_setup.bash && \
    cmake -DCH_ENABLE_MODULE_IRRLICHT=ON \
          -DCH_ENABLE_MODULE_PYTHON=ON \
          -DCH_ENABLE_MODULE_ROS=ON \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig 


# Stage 6: Final runtime image, copy built artifacts
FROM base

# Copy CMake binaries and share files from cmake-builder stage
COPY --from=cmake-builder /usr/bin/cmake /usr/bin/cmake
COPY --from=cmake-builder /usr/share/cmake-3.26 /usr/share/cmake-3.26

# Copy Boost libraries from boost-builder stage
COPY --from=boost-builder /usr/local/include /usr/local/include
COPY --from=boost-builder /usr/local/lib /usr/local/lib

# Copy chrono_ros_interfaces install from chrono-ros-builder stage
COPY --from=chrono-ros-builder /root/chrono_ros_interfaces_build/ /opt/chrono_ros_interfaces_build/

# Copy PyChrono install files from chrono-builder stage (assumed install location in /usr/local)
COPY --from=chrono-builder /usr/local /usr/local

# Extend PYTHONPATH so installed PyChrono package is found at runtime
ENV PYTHONPATH=/usr/local/share/chrono/python/:$PYTHONPATH

RUN ldconfig

RUN useradd -ms /bin/bash user && \
    echo "user:user" | chpasswd && \
    adduser user sudo && \
    echo "export PS1='\[\033[1;33m\]\u@\h\[\033[0m\]:\[\033[1;34m\]\w\[\033[0m\]\$ '" >> /home/user/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc && \
    echo "source /opt/chrono_ros_interfaces_build/install/local_setup.bash" >> /home/user/.bashrc
WORKDIR /home/user
USER user
CMD ["bash", "-l"]










