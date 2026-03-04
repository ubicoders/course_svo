#!/bin/bash

# Exit immediately if any command fails
set -e

echo "Updating apt package lists..."
sudo apt-get update

echo "Installing prerequisites..."
# curl is needed to download rustup.
# build-essential installs gcc/g++ which Cargo needs to compile many crates.
sudo apt-get install -y curl build-essential

echo "Downloading and installing Rust and Cargo via rustup..."
# The -y flag automatically selects the default, standard installation
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

echo "Sourcing the Cargo environment..."
# This makes the 'cargo' command available in the rest of this script
source "$HOME/.cargo/env"

echo "========================================"
echo "Installation Successful!"
echo "Installed version:"
cargo --version
echo "========================================"

echo ""
echo "IMPORTANT: To use Cargo in your current terminal session, run this command:"
echo "    source \$HOME/.cargo/env"
echo "Alternatively, just close this WSL window and open a new one."