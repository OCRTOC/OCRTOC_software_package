docker start ocrtoc
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ocrtoc`