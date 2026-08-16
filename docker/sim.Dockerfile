# syntax=docker/dockerfile:1

# sim과 moveit 두 서비스가 이미지 하나를 공유한다.
# ROBOTIS의 docker/Dockerfile은 실기 전용
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep \
        git \
        # --- 시뮬레이션 ---
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        # --- 제어 ---
        ros-jazzy-ros2-control \
        ros-jazzy-dynamixel-sdk \
        ros-jazzy-ros2-controllers \
        # --- 플래닝 / 시각화 ---
        ros-jazzy-moveit \
        ros-jazzy-rviz2 \
        # --- 기술 ---
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-realsense2-description \
        # --- 그래픽 유저스페이스 (2026-08-11, GT 730 머신) ---
        # 커널쪽 nouveau(DRM)는 호스트 것을 /dev/dri 로 넘겨받지만,
        # OpenGL 구현체는 컨테이너 안의 Mesa다. libgl1-mesa-dri 가 없으면
        # gallium 드라이버(nouveau)를 못 찾아 조용히 llvmpipe(CPU)로 떨어진다.
        # -> "GPU를 넘겼는데 왜 느리지"의 진범. glxinfo로 확인할 것.
        libgl1-mesa-dri \
        libglx-mesa0 \
        libegl-mesa0 \
        libglu1-mesa \
        mesa-utils \
        # --- 데모 영상 편집 (2026-08-15) ---
        # media/edited/edit.py, demo.py가 촬영 원본에 자막·배속을 얹을 때 쓴다.
        # 자막이 한국어라 CJK 글리프가 있는 폰트가 있어야 한다 - 없으면 libass가
        # 조용히 두부(□□□)로 그린다. 렌더는 성공하는데 글자만 안 보인다.
        ffmpeg \
        fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["sleep", "infinity"]
