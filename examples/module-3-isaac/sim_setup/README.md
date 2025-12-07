# Isaac Sim Setup Example

This example demonstrates setting up Isaac Sim with domain randomization for robot training.

## Prerequisites

- NVIDIA Isaac Sim 2023.1.0 or newer
- NVIDIA GPU with RTX support (2000 series+)
- Ubuntu 20.04/22.04 or Windows 10/11

## Installation

```bash
# Download from NVIDIA: https://developer.nvidia.com/isaac-sim
# Install via Omniverse Launcher

# Verify installation
~/.local/share/ov/pkg/isaac_sim-*/python.sh --version
```

## Example: Randomized Robot Training

See `randomized_training.py` for complete setup with:
- Creating parallel environments
- Domain randomization (textures, lighting, physics)
- RL training with PPO
- Synthetic data generation

## Running

```bash
# From Isaac Sim install directory
./python.sh path/to/randomized_training.py
```

## Key Concepts

**Parallel Environments**: 1024+ robots trained simultaneously
**Domain Randomization**: Varies textures, lighting, physics for robust policies
**Replicator**: Generates labeled synthetic data for vision ML
**PhysX GPU**: All physics on GPU for 100x speedup

## Outputs

- `runs/`: Trained RL policies (.pth files)
- `output/synthetic_data/`: Generated images with labels (COCO format)

## Next Steps

- Deploy trained policy on real robot with Isaac ROS
- Fine-tune with real-world data
- Monitor sim-to-real transfer metrics
