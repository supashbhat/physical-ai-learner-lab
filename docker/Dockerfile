FROM ros:humble-ros-base-jammy@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987

ENV ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=1 \
    PYTHONUNBUFFERED=1

WORKDIR /lab
COPY docker/ros2_node.py docker/ros2_smoke_test.py /lab/

CMD ["python3", "/lab/ros2_node.py"]
