sudo docker run -it \
-v "$(pwd)/../../":/workspace \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e XDG_RUNTIME_DIR=/tmp/runtime-root \
-e DISPLAY=unix$DISPLAY \
--net=host \
--privileged \
--name self_vehicle_env \
leolixingyou/ros1_for_self-driving:0.2


