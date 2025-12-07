---
sidebar_position: 3
title: "Appendix C: Setup Instructions"
description: Development environment setup for all modules
---

# Appendix C: Setup Instructions

Step-by-step setup for the development environment used in this textbook.

## Operating System

**Recommended**: Ubuntu 22.04 LTS (Jammy Jellyfish)

**Alternatives**:
- Windows 11 with WSL2 (Ubuntu 22.04)
- macOS (limited ROS 2 support via Docker)

## Module-Specific Setup

### Module 0: Physical AI Foundations

**No setup required** - conceptual module with pseudocode only.

### Module 1: ROS 2

**Install ROS 2 Humble** (Ubuntu 22.04):

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

**Source ROS 2** (add to `~/.bashrc`):
```bash
source /opt/ros/humble/setup.bash
```

### Module 2: Gazebo & Unity

**Install Gazebo Harmonic**:

```bash
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable \
  $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic
```

**Unity Setup**:
- Download Unity Hub: https://unity.com/download
- Install Unity 2021.3 LTS or later
- Add Unity Robotics Hub package from Package Manager

### Module 3: NVIDIA Isaac

**Requirements**:
- NVIDIA GPU (RTX series recommended)
- Ubuntu 20.04 or 22.04
- NVIDIA Driver 525+ and CUDA 11.8+

**Install Isaac Sim** (requires NVIDIA Omniverse):
1. Download Omniverse Launcher: https://www.nvidia.com/en-us/omniverse/download/
2. Install Isaac Sim 2023.1.1+ from Omniverse Launcher
3. Follow Isaac ROS installation: https://nvidia-isaac-ros.github.io/

**Install Isaac ROS** (Docker-based):
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Module 4: VLA Models

**Install PyTorch** (CUDA 11.8):
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

**Install Transformers**:
```bash
pip3 install transformers accelerate datasets
```

**Install OpenVLA dependencies**:
```bash
git clone https://github.com/openvla/openvla.git
cd openvla
pip3 install -e .
```

### Module 5: Capstone

**Combines all previous setups**

## Docker Alternative

**All modules** can be run using Docker (recommended for reproducibility):

```bash
# Clone book repository
git clone https://github.com/your-organization/physical-ai-book.git
cd physical-ai-book/examples/module-{N}-{name}

# Build and run Docker container
docker-compose up
```

Each example directory contains:
- `Dockerfile`: Container definition
- `docker-compose.yml`: Multi-container orchestration
- `.env.example`: Environment variables template

## Hardware Requirements

### Minimum
- CPU: 4 cores (Intel i5 / AMD Ryzen 5)
- RAM: 16 GB
- Storage: 50 GB free
- GPU: Not required for Modules 0-2

### Recommended
- CPU: 8+ cores (Intel i7 / AMD Ryzen 7)
- RAM: 32 GB
- Storage: 100 GB SSD
- GPU: NVIDIA RTX 3060+ (12GB VRAM for Modules 3-5)

## Troubleshooting

### ROS 2 not sourcing
**Solution**: Ensure `/opt/ros/humble/setup.bash` is in `~/.bashrc`

### Gazebo black screen
**Solution**: Update graphics drivers, try software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

### Isaac Sim crashes
**Solution**: Check NVIDIA driver version >= 525, VRAM >= 8GB

### PyTorch CUDA not available
**Solution**: Verify CUDA installation:
```bash
nvidia-smi  # Check driver
nvcc --version  # Check CUDA toolkit
python3 -c "import torch; print(torch.cuda.is_available())"
```

## Support

For setup issues, see:
- **ROS 2**: https://docs.ros.org/en/humble/
- **Gazebo**: https://gazebosim.org/docs
- **Isaac**: https://docs.omniverse.nvidia.com/isaacsim/
- **Book Issues**: https://github.com/your-organization/physical-ai-book/issues
