# 3D Drone Location Estimation with MATLAB

This project uses MATLAB's **Optimization Toolbox** to simulate and estimate the **3D trajectory of a flying drone** based on multilateration — a technique used in GPS and radio-based location systems.

## 📡 Project Overview

- Simulate a drone flying in a widening spiral within a 5 km × 5 km × 2 km volume for 480 seconds at a constant speed.
- Generate:
  - **P**: Positions of **N** static or moving transmitters.
  - **D**: Distances from the drone to each transmitter every second.
- Estimate the drone's position:
  - Using **fsolve** for perfect measurements.
  - Using **lsqnonlin** for noisy measurements (Gaussian noise with 10 m standard deviation).
- Visualize:
  - Actual vs. estimated drone trajectory.
  - Transmitter positions.
  - 3D animation of the flight and estimation results.

## ⚙️ Key MATLAB Tools

- `fsolve` for nonlinear system solving (exact multilateration).
- `lsqnonlin` for least-squares position estimation with noisy measurements.
- `pol2cart` and `cart2sph` for coordinate transformations.
- 3D plotting and animation for trajectory visualization.

All the processing is done in a single file.

## ✅ How to Use

1. Clone or download this repository.
2. Open MATLAB and run the main simulation scripts.
3. Adjust:
   - Number of transmitters (**N**)
   - Noise level
   - Transmitter mobility
4. Watch the drone's real vs. estimated path and analyze estimation accuracy.

## 👤 Author

David Cerezo
