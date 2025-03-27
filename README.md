# 💥 PhysicsEngine
A C++-based physics engine built from the ground up with a focus on clarity, modularity, and correctness.
Designed for educational use, simulation, and integration into custom game or research environments.
---
## 💡 Features
### Collision Detection & Resolution
- Sphere-vs-Sphere
- AABB-vs-AABB
- OBB-vs-OBB (using SAT)
- OBB-vs-AABB hybrid detection    
### Impulse Resolution
- Positional correction
- Friction and restitution (Coulomb model)
- Contact normals and separation forces    
### Physics Core
- RigidBody dynamics with linear & angular motion
- Quaternion rotation + inertia tensor support
 - External force and torque accumulation
- Integrator (semi-implicit Euler)    
### Broad Phase
- Uniform Grid-based spatial hashing for pair pruning    
### Modular Math Library
- Vector3, Matrix3, Quaternion with operator overloads
- SAT, Dot/Cross product, normalization
---
## 🧱 Project Structure
📁 docs/             — Project documentation (design notes, architecture)  
📁 scripts/           — Build scripts or tools  
📁 src/                 — Engine source files  
│   ├── AABB.*, Collision.*, Obb.*, RigidBody.*, etc.  
📁 tests/  
📁 web/           
---
## 🔧 How It Works
### RigidBody Simulation
- Each object holds state: position, velocity, orientation, mass, etc.
- Force/torque accumulation and Euler-based integration
- Quaternion rotation for 3D angular motion
### Collision System
- Contact resolution via impulse
- Supports multiple shape types
- Collision normal and penetration calculated per case
- Friction applied along tangents using clamped Coulomb model
### Broad Phase
- Uniform 3D spatial hash grid
- Reduces pairwise checks (n² → near-linear)
- Queries neighbors only
---
## 🚧 Current Work in Progress
- Constraint Solvers (Constraints.cpp)
- Physics integration layer (physics.cpp)
- Timer and profiling utilities (Timer.cpp)
- Unit tests for each module
- Web/Frontend visualisation (likely WebGL or React-based)
---  
## 🧪 Planned Tests
Each module (`AABB`, `Sphere`, `OBB`, etc.) will be tested with a dedicated test suite in the `tests/` directory,
designed to validate edge cases and compliance with real-world expectations.
