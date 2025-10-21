# Use the official ROS 2 Humble desktop image as base
FROM osrf/ros:humble-desktop

# Ensure noninteractive APT installations
ENV DEBIAN_FRONTEND=noninteractive

# 1) Create a symlink so that the command python refers to python3
RUN ln -sf "$(which python3)" /usr/local/bin/python

# 2) Install preliminary packages required for Chrono and ROS interfaces
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      swig \
      libirrlicht-dev \
      libgl1-mesa-glx \
      x11-xserver-utils \
      x11-apps \
      python3-pip \
      libbz2-dev \
      wget && \
    rm -rf /var/lib/apt/lists/*

# 3) Install CMake 3.26 from source
RUN wget https://apt.kitware.com/keys/kitware-archive-latest.asc && \
    ls -lh kitware-archive-latest.asc && \
    cat kitware-archive-latest.asc | head && \
    gpg --dearmor kitware-archive-latest.asc && \
    sudo mv kitware-archive-latest.asc.gpg /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | \
        sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    sudo apt-get update && \
    sudo apt-get install cmake=3.26.* cmake-data=3.26.* && \
    echo -e "Package: cmake\nPin: version 3.26*\nPin-Priority: 1001" | \
        sudo tee /etc/apt/preferences.d/cmake-pin && \
    echo -e "Package: cmake-data\nPin: version 3.26*\nPin-Priority: 1001" | \
        sudo tee /etc/apt/preferences.d/cmake-data-pin

# 4) Install Boost 1.86 from source
RUN git clone --recursive --branch boost-1.86.0 https://github.com/boostorg/boost.git boost-1.86.0 && \
    cd boost-1.86.0 && \
    ./bootstrap.sh --prefix=/usr/local && \
    sudo ./b2 install -j$(nproc) && \
    sudo ldconfig


# 5) Install Python packages via pip
#    * numpy pinned at 1.24.0
#    * matplotlib for plotting
#    * pytz for timezone support
#    * scipy for scientific computing
RUN python -m pip install --no-cache-dir \
      numpy==1.24.0 \
      matplotlib \
      pytz \
      scipy

# 6) Source the ROS 2 Humble environment
#    This sets up ROS_DOMAIN_ID, ROS_PACKAGE_PATH, and other env vars
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 7) Clone and build the chrono_ros_interfaces package
#    * Create a workspace directory
#    * Clone the GitHub repo into src/
#    * Build with colcon
WORKDIR /root/chrono_ros_interfaces_build
RUN mkdir -p src && \
    git clone https://github.com/projectchrono/chrono_ros_interfaces src/chrono_ros_interfaces && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 8) Clone, build, and install PyChrono (Chrono 9.0.1) from source
WORKDIR /root/chrono_build
RUN git clone https://github.com/projectchrono/chrono . && \
    mkdir -p build && \
    cd build && \
    # Ensure Python modules from build folder are discoverable
    export PYTHONPATH=/root/chrono_build/build/bin:$PYTHONPATH && \
    cmake -DENABLE_MODULE_IRRLICHT=ON \
          -DENABLE_MODULE_PYTHON=ON \
          -DENABLE_MODULE_ROS=ON \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig 

# Cleanup
WORKDIR /root

# Extend PYTHONPATH so installed PyChrono package is found at runtime
ENV PYTHONPATH=/usr/local/share/chrono/python/:$PYTHONPATH

# Default to interactive bash shell
CMD ["bash", "-l"]

