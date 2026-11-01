# ROS 2 Jazzy en el PC para hablar con la Pi del robot (misma versión que la Pi: Humble y Jazzy no se entienden por DDS).
# Trae RViz, teleop y herramientas de TF. Se usa con scripts/pc-ros.sh, que monta el repo en /wallens.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-tf2-tools \
      ros-jazzy-rqt-graph \
      ros-jazzy-rqt-tf-tree \
      libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Usuario ubuntu (uid 1000, el mismo que en el PC): lo que se cree en /wallens queda a tu nombre, no de root
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc
USER ubuntu
ENV ROS_LOG_DIR=/tmp/roslog
WORKDIR /wallens
