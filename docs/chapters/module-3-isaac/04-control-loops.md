---
sidebar_position: 4
title: "Chapter 4: Control Loops"
description: Reinforcement learning, imitation learning, and hybrid control with Isaac
tags: [reinforcement-learning, imitation-learning, isaac-gym, control]
---

# Chapter 4: Control Loops

## Learning Objectives

1. Train RL policies in Isaac Gym for parallel robot learning
2. Implement imitation learning pipelines from demonstrations
3. Design hybrid control architectures combining classical and learning-based methods

## 4.1 Reinforcement Learning in Isaac Gym

### Isaac Gym Integration

**Isaac Gym**: GPU-accelerated RL environment (part of Isaac Sim)

**Features**:
- **Parallel Environments**: 1000s of robots simultaneously
- **Throughput**: 100k steps/second (vs 100 steps/second single robot)
- **GPU Tensors**: Observations/actions stay on GPU (no CPU transfer)

**Setup**:
```python
from omni.isaac.gym.vec_env import VecEnvBase

class HumanoidEnv(VecEnvBase):
    def __init__(self, cfg, sim_device, graphics_device_id, headless):
        self.num_envs = cfg["env"]["numEnvs"]  # e.g., 4096
        self.num_obs = 108  # Observation dim
        self.num_actions = 21  # Action dim (joint targets)

        super().__init__(cfg, sim_device, graphics_device_id, headless)

    def create_sim(self):
        # Create Isaac Sim physics scene
        self.sim = gymapi.acquire_sim()
        # ... setup

    def reset(self):
        # Reset 4096 robots in parallel
        self.gym.set_actor_root_state_tensor(self.sim, self.root_states)
        return self.obs

    def step(self, actions):
        # Apply actions to all robots
        self.gym.set_dof_position_target_tensor(self.sim, actions)

        # Step physics (all robots in parallel)
        self.gym.simulate(self.sim)

        # Compute observations and rewards (on GPU)
        self.obs = self.compute_observations()
        self.rew = self.compute_rewards()
        self.done = self.compute_dones()

        return self.obs, self.rew, self.done, {}
```

### Training with PPO

**Algorithm**: Proximal Policy Optimization (stable, sample-efficient)

```python
from rl_games.algos_torch import torch_ext
from rl_games.common import env_configurations, vecenv

# Register environment
vecenv.register('IsaacHumanoid', lambda cfg: HumanoidEnv(cfg))

# Training config
config = {
    'params': {
        'seed': 42,
        'algo': {
            'name': 'a2c_continuous'
        },
        'model': {
            'name': 'continuous_a2c_logstd'
        },
        'network': {
            'name': 'actor_critic',
            'separate': False,
            'mlp': {
                'units': [256, 256, 128],
                'activation': 'elu'
            }
        },
        'config': {
            'name': 'HumanoidWalk',
            'env_name': 'IsaacHumanoid',
            'num_actors': 4096,
            'num_steps_per_env': 16,
            'minibatch_size': 32768,
            'learning_rate': 3e-4,
            'horizon_length': 16,
            'gamma': 0.99,
            'lam': 0.95
        }
    }
}

# Train
runner = Runner()
runner.load(config)
runner.run({'train': True, 'play': False, 'checkpoint': 'runs/'})
```

**Training Time**: 1 hour to achieve walking (vs 24 hours single robot)

### Reward Design

**Sparse** vs **Dense** rewards:

```python
def compute_rewards(self):
    # Dense reward (guide learning)
    forward_vel = self.root_states[:, 7]  # x velocity
    upright = self.root_states[:, 3]  # z-component of quaternion
    energy = torch.sum(torch.abs(self.joint_torques), dim=-1)

    reward = (
        2.0 * forward_vel  # Encourage forward motion
        + 1.0 * upright  # Stay upright
        - 0.001 * energy  # Minimize energy
        - 5.0 * self.fallen  # Penalty for falling
    )

    return reward
```

**Termination Conditions**:
```python
def compute_dones(self):
    # Fallen: torso below threshold
    fallen = self.root_states[:, 2] < 0.3  # z-position

    # Out of bounds
    out_of_bounds = torch.abs(self.root_states[:, :2]) > 10.0  # x, y

    # Timeout
    timeout = self.progress_buf >= self.max_episode_length

    return fallen | out_of_bounds.any(dim=-1) | timeout
```

## 4.2 Imitation Learning Pipelines

### Behavior Cloning

**Idea**: Learn policy from expert demonstrations

**Workflow**:
1. Collect expert demos (teleop, scripted, or human)
2. Train policy: `π(a|s) ≈ expert(s)`
3. Deploy

**Data Collection**:
```python
# Teleoperate robot, log state-action pairs
demos = []
while teleoperating:
    state = robot.get_state()  # Joint positions, velocities
    action = gamepad.get_input()  # Desired joint velocities
    demos.append((state, action))

# Save dataset
np.save('demos.npy', demos)
```

**Training**:
```python
import torch
import torch.nn as nn

class BehaviorClonePolicy(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim)
        )

    def forward(self, state):
        return self.net(state)

# Load demos
demos = np.load('demos.npy', allow_pickle=True)
states = torch.tensor([d[0] for d in demos])
actions = torch.tensor([d[1] for d in demos])

# Train
policy = BehaviorClonePolicy(state_dim=108, action_dim=21)
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)

for epoch in range(1000):
    pred_actions = policy(states)
    loss = nn.MSELoss()(pred_actions, actions)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
```

**Limitation**: Distribution mismatch (policy sees states expert never visited)

### DAgger (Dataset Aggregation)

**Idea**: Iteratively query expert, expand dataset

```python
# Round 1: Train on initial demos
policy.train(expert_demos)

for i in range(10):  # 10 rounds
    # Deploy policy, collect rollouts
    rollouts = []
    for episode in range(100):
        state = env.reset()
        for t in range(horizon):
            action = policy(state)  # Policy action
            expert_action = expert(state)  # Query expert
            rollouts.append((state, expert_action))  # Use expert action
            state, _, done, _ = env.step(action)
            if done:
                break

    # Augment dataset
    expert_demos.extend(rollouts)

    # Retrain
    policy.train(expert_demos)
```

**Advantage**: Policy learns to recover from its own mistakes

### Generative Adversarial Imitation Learning (GAIL)

**Idea**: Learn reward function from demos, then RL

```python
# Discriminator: real (expert) vs fake (policy)
discriminator = nn.Sequential(
    nn.Linear(state_dim + action_dim, 256),
    nn.ReLU(),
    nn.Linear(256, 1),
    nn.Sigmoid()
)

# Train discriminator
for epoch in range(epochs):
    # Expert data
    expert_s, expert_a = sample_expert_demos()
    expert_score = discriminator(torch.cat([expert_s, expert_a], dim=-1))

    # Policy data
    policy_s, policy_a = sample_policy_rollouts()
    policy_score = discriminator(torch.cat([policy_s, policy_a], dim=-1))

    # Binary cross-entropy
    loss = -torch.mean(torch.log(expert_score) + torch.log(1 - policy_score))
    # Update discriminator

# Use discriminator as reward for RL
reward = -torch.log(discriminator(s, a))  # High reward for expert-like behavior
```

## 4.3 Hybrid Control Architectures

### Classical + Learning

**Idea**: Classical control for stability, learning for adaptation

**Example** (Quadruped Locomotion):
```python
class HybridController:
    def __init__(self):
        self.classical = PIDController()  # Joint-level PD control
        self.learned = NeuralPolicy()  # Learned foothold planner

    def control(self, state):
        # Learned: High-level planning
        target_footholds = self.learned.plan_footholds(state)

        # Classical: Low-level tracking
        joint_targets = self.classical.inverse_kinematics(target_footholds)

        return joint_targets
```

**Advantages**:
- Stability guarantees from classical
- Adaptability from learning
- Interpretable (can inspect classical component)

### Residual RL

**Idea**: Learning corrects classical controller

```python
class ResidualController:
    def __init__(self):
        self.classical = PDController(kp=100, kd=10)
        self.residual = NeuralNetwork()

    def control(self, state, target):
        # Classical baseline
        classical_action = self.classical.control(state, target)

        # Learned correction
        residual_action = self.residual(state, target)

        # Combined (bound residual)
        return classical_action + 0.1 * residual_action
```

**Training**: RL trains `residual`, `classical` frozen

**Use Case**: Fine-tune PID gains per environment (rough terrain, slopes)

### Hierarchical Control

**High-level** (slow): Task planning (learned)
**Mid-level** (medium): Motion primitives (classical or learned)
**Low-level** (fast): Joint control (PD)

```python
class HierarchicalController:
    def __init__(self):
        self.high_level = TaskPlanner()  # Outputs: "walk", "grasp", "stand"
        self.mid_level = MotionPrimitives()  # Outputs: joint trajectories
        self.low_level = PDController()  # Tracks trajectories

    def control(self, state, goal):
        # High-level: Decide what to do (10 Hz)
        task = self.high_level.plan(state, goal)

        # Mid-level: Generate trajectory (100 Hz)
        trajectory = self.mid_level.execute(task, state)

        # Low-level: Track trajectory (1000 Hz)
        torques = self.low_level.track(trajectory, state)

        return torques
```

**Example**: Humanoid navigation
- High-level: A* path planning
- Mid-level: Footstep planner (learned)
- Low-level: Joint PD control

## Exercises

**Exercise 4.1**: RL Training in Isaac Gym
- Implement a cartpole balancing task in Isaac Gym (4096 parallel environments)
- Train a PPO policy for 1000 episodes
- Experiment with reward shaping (balance angle, velocity penalties)
- Plot learning curves and analyze convergence

**Exercise 4.2**: Behavior Cloning for Grasping
- Collect 500 expert demonstrations of pick-and-place in Isaac Sim (teleoperation or scripted)
- Train behavior cloning policy (vision → gripper pose)
- Evaluate on test objects not seen during training
- Measure success rate and failure modes

**Exercise 4.3**: DAgger Interactive Learning
- Start with initial BC policy from Exercise 4.2
- Implement DAgger: deploy policy, collect expert corrections, retrain
- Run 5 iterations of DAgger
- Plot success rate vs iteration and compare with pure BC

**Exercise 4.4**: Hybrid Residual Controller
- Implement a PD controller for quadruped standing balance
- Add learned residual correction for terrain adaptation
- Train residual policy on randomized terrain in Isaac Sim
- Compare performance: PD-only vs PD+residual on slopes/stairs

## Summary

**RL in Isaac Gym**: Parallel training (1000s of robots), PPO for stability, reward engineering
**Imitation Learning**: Behavior cloning (fast, brittle), DAgger (iterative), GAIL (adversarial)
**Hybrid Control**: Classical + learning, residual RL, hierarchical architectures

**Module 3 Complete!** Next modules cover VLA models (Module 4) and capstone project (Module 5).
