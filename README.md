# 💥 PhysicsEngine

A C++-based modular physics engine built from the ground up with a focus on clarity, correctness, and real-time simulation. Designed for educational use, integration into custom engines, or research-driven physics systems.

```md
![C++](https://img.shields.io/badge/language-C++17-blue.svg)
![Status](https://img.shields.io/badge/status-WIP-orange.svg)
![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)
```
## 💡 Features

### Collision Detection & Resolution
- Sphere-vs-Sphere
- AABB-vs-AABB
- OBB-vs-OBB (via Separating Axis Theorem)
- Hybrid OBB-vs-AABB support

### Impulse Resolution
- Positional correction
- Restitution and friction (Coulomb model with clamped tangents)
- Directional contact normals and separation resolution

### Physics Core
- RigidBody dynamics (linear & angular)
- Quaternion-based rotation
- Inertia tensor support
- External force & torque accumulation
- Semi-implicit Euler integration

### Broad Phase Optimization
- Uniform Grid-based spatial hashing
- Efficient pruning of potential collision pairs

### 🧮 Math Library
- `Vector3`, `Matrix3`, `Quaternion` with C++ operator overloads
- Dot, cross, normalization, SAT support

---
## 🧱 Project Structure

```text
PhysicsEngine/
├── docs/            # Project documentation (design notes, architecture)
├── scripts/         # Build scripts or tools
├── src/             # Engine source files (AABB.*, Collision.*, Obb.*, etc.)
├── tests/           # Unit tests (modular)
├── web/             # Optional frontend visualiser
├── .gitignore
├── LICENSE
└── README.md
```


---
## 🔧 How It Works

### RigidBody Simulation
- Tracks position, velocity, angular state, mass
- Accumulates forces & torques per frame
- Semi-implicit Euler integration
- Rotation via quaternions

### Collision System
- Calculates penetration & normals per shape
- Handles multiple collision types dynamically
- Friction + restitution resolved per frame
- Works with static & dynamic rigidbodies

### Broad Phase
- Hash-based grid for spatial partitioning
- 27-cell neighbor checks for localized collisions
- Reduces brute-force pairwise checks (`O(n²)` → `O(n)`ish)

---

## 🚧 Work in Progress

- ⛓ Constraint solver (hinges, springs, fixed joints)
- 🧮 Physics integration core (`physics.cpp`)
- ⏱ Timer utilities for profiling
- 🔬 Unit tests (AABB, Sphere, OBB, etc.)
- 🖥 Web frontend visualization (WebGL or React)

---
## 🧪 Unit Testing

The `tests/` folder contains modular unit tests for each major component:

- `test_collision.cpp`: Verifies all shape collision interactions and impulse responses.
- `test_rigidbody.cpp`: Validates RigidBody integration, force accumulation, and mass-based behavior.
- `test_mathutils.cpp`: Ensures correctness for Vector, Quaternion, and Matrix operations.
- `test_uniformgrid.cpp`: Confirms spatial hashing, cell assignment, and broad-phase pair detection.
- `test_world.cpp`: End-to-end physics stepping and scene integration.
- `test_constraints.cpp`: Placeholder for upcoming constraint solver testing.

All tests are managed through CMake and will be expanded as new modules are developed.


