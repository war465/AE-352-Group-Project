# AE 352 Group 24: Quadcopter Digital Twin

A 6-DOF digital twin of a Holybro X500 V2 quadcopter (4S 5200 mAh). The only
inputs to the model are the four **motor torques** (ER2). The model flies the
three performance goals from the project statement.

## Quick start

```bash
pip install -r requirements.txt
jupyter lab drone_twin.ipynb      # then Run -> Run All Cells
```

Running all cells simulates PG1-PG3, checks the requirements, shows the plots, and runs the
physics sanity tests. Figures are saved to `figures/` (PDF + PNG) and metrics to
`results/summary.json`. The missions take about 2-3 minutes in total.

## Notebook layout

| Section | Contents |
|---|---|
| 1. Physical parameters | Mass, inertia estimate, propeller, motor and battery constants from the bill of materials |
| 2. Rotation kinematics | ZYX Euler rotation matrix and Euler-rate matrix |
| 3. Equations of motion | 17-state model, RK4 integrator, ground contact, motor/battery electrical model |
| 4. Reference trajectories | Min-jerk lines, yaw turns, circle, constant-speed descent |
| 5. Flight controller | Position PID → attitude PD → mixer → ESC speed loop → motor torque |
| 6. Simulator | Fixed-step closed-loop simulation with logging |
| 7. Missions | PG1/PG2/PG3 definitions and requirement checks |
| 8. Plotting | Report figures (all DOFs vs time, rotor speeds, motor torques, battery, flight path) |
| 9. Run the performance goals | One cell per mission, plus a summary |
| 10. Physics sanity tests | Unit tests of the equations of motion, controller and trajectories |

## Model summary

State `x = [r (3), v (3), φ θ ψ (3), p q r (3), Ω₁..Ω₄ (4), q_used (1)]`:

- **Translation:** `m v̇ = R(φ,θ,ψ)[0,0,ΣTᵢ]ᵀ − m g e_z − ½ρ C_dA |v|v`, with `Tᵢ = k_T Ωᵢ²`
- **Attitude kinematics:** `[φ̇ θ̇ ψ̇]ᵀ = W(φ,θ) ω` (ZYX Euler angles)
- **Rotation:** `I ω̇ = M_thrust − Σ sᵢ τᵢ e_z − ω×Iω − ω×h_rotors`
- **Rotors (ER2 input):** `J_r Ω̇ᵢ = τᵢ − k_Q Ωᵢ² − τ_f`
- **Battery:** `q̇_used = I_batt(τ, Ω, SOC)` from the motor back-EMF/resistance model

The body frame is FLU (x forward, y left, z up). Motor numbering follows PX4
quad-X: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.

## Assumptions to verify (Procurement / QC)

These values in section 1 of the notebook are marked `ASSUMED` and are not on a datasheet:
propeller `C_T = 0.11` and `C_P = 0.045` (check against Tyto database data for
the X500 10" prop), motor winding resistance, battery internal resistance,
drag area, and the frame mass split used for the inertia estimate.
