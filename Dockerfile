FROM registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_base_image:0.0.2

# 設定 ROS 版本
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

WORKDIR /workspaces

# 初始化 rosdep 並安裝相依套件
RUN apt update && \
    rosdep update --rosdistro $ROS_DISTRO && \
    colcon mixin update && \
    colcon metadata update && \
    rosdep install -q -y -r --from-paths src --ignore-src && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --mixin release && \
    source ./install/setup.bash

# 安裝 Python 依賴套件
RUN pip3 install --no-cache-dir \
    gymnasium \
    stable-baselines3 \
    numpy==2.0.1

# 設定 ENTRYPOINT
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "/bin/bash", "-l" ]
