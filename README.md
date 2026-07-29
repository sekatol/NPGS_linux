# NPGS_linux

Ported from `baopinshui/NPGS`

## Arch Linux

```bash
# Vulkan
sudo pacman -S vulkan-headers vulkan-icd-loader vulkan-tools

# Window
sudo pacman -S glfw-wayland   # Wayland
# sudo pacman -S glfw-x11     # X11

# Math
sudo pacman -S glm

# Logger
sudo pacman -S spdlog

# JSON
sudo pacman -S nlohmann-json

# Shader compiler
sudo pacman -S shaderc

# Fonts
sudo pacman -S freetype2

# SPIR-V
sudo pacman -S spirv-cross

# assimp & boost
sudo pacman -S assimp boost
```

## Build

```bash
# Configure
cd NPGS_linux
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug # For release: cmake .. -DCMAKE_BUILD_TYPE=Release

# Compile shaders
cd ..
python shaders/compile.py
cp assets/Shaders/*.spv build/assets/Shaders/

# Compile application
cd build
cmake --build . -j8

# Launch
./NPGS_linux
```
