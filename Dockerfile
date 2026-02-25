# =============================================================================
# forklift - ROS 2 Jazzy + Gazebo Sim (Tricycle robot simulation)
# =============================================================================
# Base: desktop-full ile RViz2, rqt ve GUI araçları dahil; simülasyon için
# Gazebo Sim (ros_gz_sim) ve proje bağımlılıkları ekleniyor.
# =============================================================================

FROM osrf/ros:jazzy-desktop-full

# Apt önbellek ve etkileşimli soruları kapat (CI/Docker uyumu)
ENV DEBIAN_FRONTEND=noninteractive

# 1) Derleme araçları
#    - build-essential, cmake: C++ paketleri (offset_tricycle_controller)
#    - python3-colcon-common-extensions: colcon build
#    - python3-rosdep: bağımlılık çözümü (isteğe bağlı kullanım)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2) ROS Jazzy paketleri (proje package.xml + launch bağımlılıkları)
#    Simülasyon: ros_gz_sim, ros_gz_bridge, gz_ros2_control
#    Kontrol: controller_manager, joint_state_broadcaster, robot_state_publisher
#    EKF: robot_localization | Teleop: joy | Mesajlar: ackermann_msgs, sensor_msgs
#    offset_tricycle_controller için: generate_parameter_library, backward_ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-robot-localization \
    ros-jazzy-joy \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-controller-manager \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-sensor-msgs \
    ros-jazzy-generate-parameter-library \
    ros-jazzy-backward-ros \
    ros-jazzy-ros2-control \
    ros-jazzy-realtime-tools \
    ros-jazzy-rcpputils \
    && rm -rf /var/lib/apt/lists/*

# 3) Workspace dizini (root kullanıcı evinin altında)
WORKDIR /root/ros2_ws

# 4) Kaynak kodu kopyala ( .dockerignore ile build/install/log atlanır )
COPY src ./src

# 5) Colcon ile derleme (sadece offset_tricycle_robot + offset_tricycle_controller;
#    tricycle_robot harici tricycle_controller'a bağlı olduğu için atlanır)
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-up-to offset_tricycle_robot --cmake-args -DCMAKE_BUILD_TYPE=Release

# 6) Giriş noktası: ROS ortamı + workspace kaynaklı, bash açık kalır
#    Konteyner içinde: source /root/ros2_ws/install/setup.bash && ros2 launch ...
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && \"$@\"", "--"]
CMD ["bash"]
