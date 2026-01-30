# 3-DOF Torsional Vibration Analysis – Automotive Drivetrain  
**Effect of Clutch Damper Stiffness on Modal Properties & Dynamic Response**

This repository contains my individual MATLAB implementation for the modal analysis and undamped forced response of a simplified 3 degree-of-freedom (3-DOF) lumped-parameter torsional model of an automotive drivetrain (engine/flywheel – clutch disc – transmission input shaft).

The main focus of the project is to investigate how **clutch damper stiffness** influences:
- Natural frequencies
- Mode shapes
- Orthogonality and mass-normalization behavior
- Time-domain torsional vibration response under harmonic engine excitation

## Model Description
- **3 torsional degrees of freedom**
- Inertias (kg·m²):  
  - J₁ = 0.15 (Engine + Flywheel)  
  - J₂ = 0.02 (Clutch disc)  
  - J₃ = 0.05 (Transmission input shaft)
- Torsional stiffnesses compared in two cases:  
  - **High stiffness damper**: k₁ = 500 Nm/rad  
  - **Low stiffness damper**: k₁ = 100 Nm/rad  
  - Fixed shaft stiffness: k₂ = 5000 Nm/rad

## Key Features of the Code
- Assembly of mass (inertia) matrix [M] and stiffness matrix [K]
- Generalized eigenvalue problem using `eig(K, M)` to obtain natural frequencies and mass-normalized mode shapes
- Detailed verification of mass-normalization (Φᵀ[M]Φ = I) and stiffness orthogonality (Φᵀ[K]Φ = Λ)
- Comparison of natural frequencies between high and low clutch stiffness cases
- Analytical solution for undamped forced response in modal (principal) coordinates
- Comprehensive visualization:
  - Individual and overlaid mode shapes
  - Bar chart comparing natural frequencies (with idling frequency reference ~13.3 Hz)
  - Time-domain angular displacement responses for each component
  - Principle coordinate time histories

## How to Run
1. Clone or download this repository
2. Open MATLAB
3. Set the current working directory to this repository folder
4. Run the main script:

   ```matlab
   torsional_vibration_analysis
