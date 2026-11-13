# Shared runtime selection for the root launchers; source after setting project_dir.
image=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["isaac_image"])' \
  "$project_dir/configs/rm27_asm2_collision.json")
docker_command=(docker)
if [[ -z ${DOCKER_CONTEXT:-} && ${DOCKER_HOST:-unix:///var/run/docker.sock} == unix:///var/run/docker.sock \
      && ! -w /var/run/docker.sock ]]; then
  docker_command=(sudo -n docker)
fi
# Keep the calling user's UID/GID while making the container home writable.
container_home_mount=(--tmpfs "/home/ubuntu:uid=$(id -u),gid=$(id -g),mode=0700")
