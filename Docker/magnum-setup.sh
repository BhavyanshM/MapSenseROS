#! /bin/sh



# Uncomment for Dear ImGui Integration and Configure with the ImGui Repository


git clone git://github.com/mosra/magnum-integration && cd magnum-integration && cd src/MagnumExternal && git clone https://github.com/ocornut/imgui.git && mv imgui ImGui && cd ../../


sed -i "s/IMGUI=OFF/IMGUI=ON/g" package/debian/rules
sed -i "s/imgui_demo/imgui_demo imgui_tables/g" modules/FindImGui.cmake

sudo nano package/debian/rules # add -DWITH_IMGUI=ON to package/debian/rules
nano modules/FindImGui.cmake # To add imgui_tables to the foreach(---)

# ########### CONTINUE AFTER ENABLING IMGUI (package/debian/rules) #########

ln -s package/debian .
dpkg-buildpackage

sudo dpkg -i ../magnum-integration*.deb


