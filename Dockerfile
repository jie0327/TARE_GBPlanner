# syntax=docker/dockerfile:1

# Build the official CMU TARE workspace from the checked-out sources in this
# directory. ros:noetic-ros-base is multi-architecture; the OR-Tools binary
# bundled by TARE is replaced with Google's official arm64 release when the
# image target is arm64.
FROM ros:noetic-ros-base AS builder

SHELL ["/bin/bash", "-c"]

ARG TARGETARCH
ARG CATKIN_JOBS=2
ARG OR_TOOLS_ARM64_URL=https://github.com/google/or-tools/releases/download/v9.8/or-tools_arm64_debian-11_cpp_v9.8.3296.tar.gz

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -o Acquire::Retries=5 update && \
    apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    bluez \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    gazebo11 \
    joystick \
    libapr1-dev \
    libeigen3-dev \
    libgazebo11-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libpcl-dev \
    libusb-1.0-0-dev \
    libusb-dev \
    pkg-config \
    python3-bluez \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
    python3-opencv \
    python3-tk \
    qtbase5-dev \
    ros-noetic-diagnostic-aggregator \
    ros-noetic-diagnostic-updater \
    ros-noetic-eigen-conversions \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-message-filters \
    ros-noetic-pcl-ros \
    ros-noetic-roslint \
    ros-noetic-rviz \
    ros-noetic-spacenav-node \
    ros-noetic-tf \
    ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/tare_planner
COPY src ./src
COPY third_party ./third_party
COPY docker/entrypoint.sh /usr/local/bin/tare-entrypoint
RUN chmod +x /usr/local/bin/tare-entrypoint

# FAST-LIO2 declares the official ROS1 Livox message package even when its
# input is standard Velodyne PointCloud2. Install Livox's official SDK first.
RUN cmake -S third_party/Livox-SDK -B /tmp/livox-sdk-build \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/livox-sdk-build -j"${CATKIN_JOBS}" && \
    cmake --install /tmp/livox-sdk-build && \
    test -f /usr/local/lib/liblivox_sdk_static.a && \
    rm -rf /tmp/livox-sdk-build

# The official TARE repository includes an amd64 OR-Tools binary. The CMU
# README documents this exact arm64 release and asks users to replace its
# include/lib directories, so do that during the image build.
RUN arch="${TARGETARCH:-$(uname -m)}"; \
    case "$arch" in \
      arm64|aarch64) \
        mkdir -p /tmp/or-tools && \
        curl --fail --location --retry 4 --retry-delay 3 \
          "$OR_TOOLS_ARM64_URL" -o /tmp/or-tools.tar.gz && \
        tar -xzf /tmp/or-tools.tar.gz -C /tmp/or-tools --strip-components=1 && \
        rm -rf src/tare_planner/src/tare_planner/or-tools/include \
               src/tare_planner/src/tare_planner/or-tools/lib && \
        cp -a /tmp/or-tools/include src/tare_planner/src/tare_planner/or-tools/ && \
        cp -a /tmp/or-tools/lib src/tare_planner/src/tare_planner/or-tools/ && \
        rm -rf /tmp/or-tools /tmp/or-tools.tar.gz ;; \
      amd64|x86_64) ;; \
      *) echo "Unsupported Docker target architecture: $arch" >&2; exit 1 ;; \
    esac; \
    test -f src/tare_planner/src/tare_planner/or-tools/lib/libortools.so

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make livox_ros_driver_generate_messages_cpp -j"${CATKIN_JOBS}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF && \
    catkin_make fast_lio_generate_messages_cpp -j"${CATKIN_JOBS}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF && \
    catkin_make -j"${CATKIN_JOBS}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF

RUN source /opt/ros/noetic/setup.bash && \
    source /opt/tare_planner/devel/setup.bash && \
    rospack find tare_planner && \
    rospack find fast_lio && \
    rospack find vehicle_simulator && \
    test -x /opt/tare_planner/devel/lib/fast_lio/fastlio_mapping && \
    test -x /opt/tare_planner/devel/lib/tb3_tare_sim/fast_lio_tare_adapter && \
    test -x /opt/tare_planner/devel/lib/tare_planner/tare_planner_node && \
    test -x /opt/tare_planner/devel/lib/vehicle_simulator/vehicleSimulator


# Keep the runtime image independent from the FAST-LIO/EGO image used by the
# larger development stack. Build tools and headers stay in the builder.
FROM ros:noetic-ros-base AS runtime

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -o Acquire::Retries=5 update && \
    apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    bluez \
    gazebo11 \
    joystick \
    libgoogle-glog0v5 \
    libgl1-mesa-glx \
    libglu1-mesa \
    python3-bluez \
    python3-matplotlib \
    python3-numpy \
    python3-tk \
    ros-noetic-diagnostic-aggregator \
    ros-noetic-diagnostic-updater \
    ros-noetic-eigen-conversions \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-message-filters \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    ros-noetic-spacenav-node \
    ros-noetic-tf \
    ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/tare_planner/src /opt/tare_planner/src
COPY --from=builder /opt/tare_planner/devel /opt/tare_planner/devel
COPY --from=builder /usr/local/bin/tare-entrypoint /usr/local/bin/tare-entrypoint

ENV ROS_PACKAGE_PATH=/opt/tare_planner/src:/opt/ros/noetic/share \
       CMAKE_PREFIX_PATH=/opt/tare_planner/devel:/opt/ros/noetic \
       LD_LIBRARY_PATH=/opt/tare_planner/src/tare_planner/src/tare_planner/or-tools/lib:/opt/tare_planner/devel/lib:/opt/ros/noetic/lib \
       GAZEBO_PLUGIN_PATH=/opt/tare_planner/devel/lib:/opt/ros/noetic/lib \
       ROS_MASTER_URI=http://127.0.0.1:11311

WORKDIR /opt/tare_planner
ENTRYPOINT ["/usr/local/bin/tare-entrypoint"]
CMD ["bash"]
