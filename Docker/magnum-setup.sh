#! /bin/sh

sudo apt-get install -y libopenal-dev libglfw3-dev libsdl2-dev libglm-dev

mkdir Temp
cd Temp

git clone git://github.com/mosra/corrade && cd corrade
ln -s package/debian .
dpkg-buildpackage
sudo dpkg -i ../corrade*.deb
cd ..

git clone git://github.com/mosra/magnum && cd magnum
ln -s package/debian .
dpkg-buildpackage
sudo dpkg -i ../magnum*.deb
cd ..

# Uncomment for Dear ImGui Integration and Configure with the ImGui Repository


git clone git://github.com/mosra/magnum-integration && cd magnum-integration
cd src/MagnumExternal
git clone https://github.com/ocornut/imgui.git
mv imgui ImGui
cd ../../
pwd


sudo nano package/debian/rules # add -DWITH_IMGUI=ON to package/debian/rules
nano modules/FindImGui.cmake # To add imgui_tables to the foreach(---)

# ########### CONTINUE AFTER ENABLING IMGUI (package/debian/rules) #########

ln -s package/debian .
dpkg-buildpackage

sudo dpkg -i ../magnum-integration*.deb


