#!/bin/bash
# 1. Install necessary drivers and diagnostic tools
sudo apt-get update
sudo apt install xdg-utils -y
sudo apt install mesa-utils mesa-vulkan-drivers vulkan-tools -y


# 2. Add GPU bridge variables to .bashrc for persistence
# We use GALLIUM_DRIVER to force the D3D12 bridge (Windows GPU)
# and VK_ICD_FILENAMES to ensure Vulkan doesn't default to the CPU (llvmpipe)
if ! grep -q "GALLIUM_DRIVER=d3d12" ~/.bashrc; then
  echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
  echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc
fi

# 3. Reload environment
source ~/.bashrc

# 4. Verify the setup
echo "--- Checking OpenGL  ---"
glxinfo -B | grep -E "Device|Accelerated"

echo "--- Checking Vulkan  ---"
vulkaninfo | grep "Vulkan Instance Version"
vkcube

echo "run source ~/.bashrc to apply the changes, then you can run the virtual robots with GPU acceleration."

