# Download necessary matertials for Mujoco
# sudo apt-get install -y python3-pip curl
MUJOCO_MATERIALS_BINARY=/root/ocrtoc_ws/ocrtoc_materials_mujoco
pip3 install gdown

echo $
if test -d "$MUJOCO_MATERIALS_BINARY"; then
    echo "$MUJOCO_MATERIALS_BINARY exists. skip downloading"
else 
    echo "Download Mujoco materials to $MUJOCO_MATERIALS_BINARY"
    gdown https://drive.google.com/uc?id=1gHRQP4H34DsoKCXlOLB-c7AiAu27JNqm -O /root/ocrtoc_ws/ocrtoc_materials_mujoco.zip
    # Extract the package to 
    echo "Extract Mujoco materials to ocrtoc_ws"
    unzip /root/ocrtoc_ws/ocrtoc_materials_mujoco.zip -d  /root/ocrtoc_ws
    rm /root/ocrtoc_ws/ocrtoc_materials_mujoco.zip
    echo "Finish extract  Mujoco materials package"
fi