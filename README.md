# Dynamic_Library

`Dynamic_Library` is an advanced Object-Oriented Programming (OOP) toolbox implemented in MATLAB for the automated derivation, calculation, and numerical resolution of the Equations of Motion (EDM) for holonomic multi-body mechanical systems. 

Developed at the **Universidad Pública de Navarra (UPNA)** as part of a Master's Thesis in Industrial Engineering, this library streamlines the transition from physical rigid-body definitions to high-performance numerical simulation and 3D visualization.

---

## 🚀 Key Features

- **Object-Oriented Architecture:** Fully structured using MATLAB namespaces to prevent conflicts with native functions and maintain clean code guidelines.
- **Symbolic & Numerical Kinematics:** Define complex kinematic chains using custom reference bases, transformation frames, geometric points, and rigid bodies.
- **Automated EDM Generation:** Calculates velocity partials, generalized active and inertia forces, mass matrices $[A]$, and their explicit time-derivatives $\dot{[A]}$.
- **State-Space Reduction:** Automatically reduces second-order differential equations into first-order systems ready for numerical integration.
- **Efficient Function-Handle Export:** Automatically generates optimized `.m` files inside a `fun_handles/` folder for fast numerical evaluation, minimizing computational overhead during loop iterations.
- **Blender 3D Animation Pipeline:** Includes utilities to export 4x4 transformation matrices to `.csv` format, allowing realistic 3D physics rendering in Blender via an included Python script.

---

## 📂 Architecture & Namespace Structure

The library uses a highly modular structure based on standard mechanical concepts:

### Core Classes Breakdown
- **`System`:** The orchestration layer. Users interact primarily with this class to instantiate coordinates, add components, and compile the system equations.
- **`Base`:** Handles coordinate bases. Supports canonical frames and angular orientation parameterizations such as Euler 1-2-3 and Euler 2-3.
- **`Point`:** Manages spatial positioning, vectors relative to other points, and automatic velocity calculations.
- **`Rigid_Body`:** Encapsulates the rigid body properties. Stores mass, 3x3 inertia tensors, reference bases, and centers of gravity ($G$).
- **`Action`:** Models internal/external interactions. Supports gravitational fields, actuators (input forces/torques), viscous damping, and Coulomb friction.

---
