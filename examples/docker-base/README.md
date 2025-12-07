# Docker Base Images

This directory contains base Docker images for running code examples from the Physical AI & Humanoid Robotics textbook.

## Available Images

### 1. ROS 2 Humble (`Dockerfile.ros2`)

**Purpose**: Run ROS 2 examples (Module 1)

**Requirements**:
- Docker 20.x+
- 4GB+ RAM
- No GPU required

**Build**:
```bash
docker build -t physical-ai-book/ros2:humble -f Dockerfile.ros2 .
```

**Run**:
```bash
docker run -it --rm \
  -v $(pwd)/../ros2-examples:/ros2_ws/src \
  physical-ai-book/ros2:humble
```

**Included Software**:
- ROS 2 Humble Desktop Full
- Gazebo (Classic, for compatibility)
- Python 3.10+
- Colcon build tools

---

### 2. NVIDIA Isaac Sim (`Dockerfile.isaac`)

**Purpose**: Run Isaac Sim/Gym/ROS examples (Module 3)

**Requirements**:
- NVIDIA GPU (Compute Capability 7.0+, e.g., RTX 2060 or higher)
- NVIDIA Container Toolkit installed
- 16GB+ VRAM (recommended)
- 32GB+ system RAM

**Build**:
```bash
docker build -t physical-ai-book/isaac:2023.1 -f Dockerfile.isaac .
```

**Run**:
```bash
docker run -it --rm \
  --gpus all \
  -v $(pwd)/../isaac-examples:/isaac_ws \
  physical-ai-book/isaac:2023.1
```

**Included Software**:
- NVIDIA Isaac Sim 2023.1.1
- PyTorch 2.0+
- CUDA 12.x + cuDNN

---

### 3. VLA Models (`Dockerfile.vla`)

**Purpose**: Run OpenVLA and SmolVLA examples (Module 4)

**Requirements**:
- NVIDIA GPU
- 16GB+ VRAM (OpenVLA 7B)
- 4GB+ VRAM (SmolVLA 450M)
- 64GB+ system RAM (for OpenVLA)

**Build**:
```bash
docker build -t physical-ai-book/vla:latest -f Dockerfile.vla .
```

**Run (OpenVLA)**:
```bash
docker run -it --rm \
  --gpus all \
  -v $(pwd)/../vla-examples:/vla_ws \
  -v ~/.cache/huggingface:/vla_ws/.cache/huggingface \
  physical-ai-book/vla:latest
```

**Included Software**:
- PyTorch 2.1.0 with CUDA 12.1
- Hugging Face Transformers 4.35+
- Accelerate (for distributed inference)

---

## License

All Dockerfiles and scripts in this directory are licensed under **Apache License 2.0**.

See [LICENSE.code.md](../../LICENSE.code.md) for full license text.

---

## Troubleshooting

### NVIDIA Container Toolkit Not Found

Install NVIDIA Container Toolkit:
```bash
# Ubuntu/Debian
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Out of VRAM Errors

For OpenVLA, reduce batch size or use SmolVLA (4GB VRAM) instead.

For Isaac Sim, reduce the number of parallel environments in Isaac Gym.

---

**Questions?** Open an issue on [GitHub](https://github.com/your-organization/physical-ai-book/issues).
