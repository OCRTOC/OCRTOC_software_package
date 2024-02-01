export docker_image=$1
echo "Create container $docker_image"
sudo docker rm -f ocrtoc
sudo docker run -i -d --gpus all --name ocrtoc --network host \
        --privileged -v /dev:/dev -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/ocrtoc_software_package:/root/ocrtoc_ws \
        -v $HOME/Desktop/docker:/root/upload \
        $docker_image
sudo xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ocrtoc`
