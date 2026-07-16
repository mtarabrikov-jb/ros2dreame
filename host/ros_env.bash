# Auto-sourced for every non-interactive shell in the container via BASH_ENV (set in
# docker-compose.yml). `docker exec ... bash -c '...'` runs a non-interactive shell
# that does NOT read .bashrc, so without this every command would need an explicit
# `source /opt/ros/jazzy/setup.bash`. Mounted (not baked in) so it applies on
# `make up` with no image rebuild.
source /opt/ros/jazzy/setup.bash
if [ -f /opt/explore_ws/install/setup.bash ]; then
    source /opt/explore_ws/install/setup.bash
fi
