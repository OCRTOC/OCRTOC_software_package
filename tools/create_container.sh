export docker_image=$1
echo "Create container $docker_image"
docker rm -f ocrtoc
docker run -i -d --gpus all --name ocrtoc --network host \
        --privileged -v /dev:/dev -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/OCRTOC_software_package:/root/ocrtoc_ws \
        -v $HOME/Desktop/docker:/root/upload \
        $docker_image
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ocrtoc`
