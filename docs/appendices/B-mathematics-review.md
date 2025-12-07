---
sidebar_position: 2
title: "Appendix B: Mathematics Review"
description: Essential mathematical concepts for robotics
---

# Appendix B: Mathematics Review

Key mathematical concepts used throughout this textbook.

## Linear Algebra

### Vectors and Matrices

**Vector**: Ordered list of numbers representing position, velocity, or direction.
```
v = [x, y, z]ᵀ
```

**Matrix**: 2D array for transformations and linear systems.
```
A = [a₁₁ a₁₂]
    [a₂₁ a₂₂]
```

**Key Operations**:
- Dot product: `v · w = vᵀw = Σ vᵢwᵢ`
- Matrix multiplication: `(AB)ᵢⱼ = Σₖ Aᵢₖ Bₖⱼ`
- Transpose: `(Aᵀ)ᵢⱼ = Aⱼᵢ`
- Inverse: `AA⁻¹ = I`

### Coordinate Transformations

**Rotation Matrix (2D)**:
```
R(θ) = [cos θ  -sin θ]
       [sin θ   cos θ]
```

**Homogeneous Transformation (3D)**:
```
T = [R  t]  (4×4 matrix)
    [0  1]
```
Where R is 3×3 rotation, t is 3×1 translation.

**Used in**: Robot kinematics, sensor fusion, SLAM

## Calculus

### Derivatives

**Definition**: Rate of change
```
f'(x) = lim[h→0] (f(x+h) - f(x))/h
```

**Rules**:
- Power rule: `d/dx(xⁿ) = nxⁿ⁻¹`
- Chain rule: `d/dx f(g(x)) = f'(g(x))·g'(x)`
- Product rule: `d/dx(fg) = f'g + fg'`

**Gradient** (multivariable):
```
∇f = [∂f/∂x₁, ∂f/∂x₂, ..., ∂f/∂xₙ]ᵀ
```

**Used in**: Optimization, neural network training, trajectory planning

### Integration

**Numerical integration** (Euler method):
```
x(t+Δt) ≈ x(t) + ẋ(t)Δt
```

**Used in**: Simulating dynamics, dead reckoning, Kalman filters

## Probability

### Probability Distributions

**Gaussian (Normal) Distribution**:
```
p(x) = (1/√(2πσ²)) exp(-(x-μ)²/(2σ²))
```
- μ: mean
- σ²: variance

**Multivariate Gaussian**:
```
p(x) = (1/√((2π)ⁿ|Σ|)) exp(-½(x-μ)ᵀΣ⁻¹(x-μ))
```
- Σ: covariance matrix

### Bayes' Theorem

```
p(A|B) = p(B|A)p(A) / p(B)
```

**Robotics form** (belief update):
```
p(state|sensor) ∝ p(sensor|state) × p(state)
                  [likelihood]    [prior]
```

**Used in**: Sensor fusion, localization, mapping, perception

### Expectation and Variance

**Expectation**: `E[X] = Σ xᵢ p(xᵢ)` or `∫ x p(x) dx`

**Variance**: `Var[X] = E[(X - E[X])²] = E[X²] - (E[X])²`

**Used in**: Kalman filters, uncertainty quantification

## Optimization

### Gradient Descent

Update rule:
```
θ ← θ - α∇L(θ)
```
- α: learning rate
- L: loss function

**Variants**:
- SGD: Stochastic (mini-batch) gradient descent
- Adam: Adaptive moment estimation
- RMSprop: Root mean square propagation

**Used in**: Neural network training, policy optimization

### Least Squares

Minimize: `||Ax - b||²`

**Closed-form solution**: `x* = (AᵀA)⁻¹Aᵀb`

**Used in**: Sensor calibration, trajectory fitting, SLAM

## Useful Identities

**Matrix Inverse Lemma** (Sherman-Morrison-Woodbury):
```
(A + UVᵀ)⁻¹ = A⁻¹ - A⁻¹U(I + VᵀA⁻¹U)⁻¹VᵀA⁻¹
```

**Used in**: Kalman filter update (efficient covariance inversion)

**Rotation Composition**:
```
R(θ₁ + θ₂) = R(θ₁)R(θ₂)
```

**Used in**: TF tree composition, multi-link robots

## Quick Reference

| Concept | Notation | Use Case |
|---------|----------|----------|
| Position vector | `p = [x, y, z]ᵀ` | Robot location |
| Rotation matrix | `R ∈ SO(3)` | Orientation |
| Velocity | `v = dp/dt` | Motion control |
| Acceleration | `a = dv/dt` | Dynamics |
| Jacobian | `J = ∂f/∂x` | Kinematics |
| Covariance | `Σ = E[(x-μ)(x-μ)ᵀ]` | Uncertainty |

## Further Reading

- **Linear Algebra**: Strang, G. (2016). *Introduction to Linear Algebra*
- **Probability**: Thrun et al. (2005). *Probabilistic Robotics*, Appendix B
- **Optimization**: Boyd & Vandenberghe (2004). *Convex Optimization*
