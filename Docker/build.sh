distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update -y
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker


# Visual Studio Code (Optional)
# Install Visual Studio Code for C++ Development (https://linuxize.com/post/how-to-install-visual-studio-code-on-ubuntu-20-04/)
# wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
# sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
# sudo apt install -y code


mkdir -p Shared_Volume #To store ROSBags for testing MapSense

docker build -t ihmcrobotics/mapsense-nvidia-ros .