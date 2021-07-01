docker start ocrtoc
sudo xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ocrtoc`