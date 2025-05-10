# gravity_sim

This a personal project that makes me experiment with physics in python.

## version history
### gravity_sim.py (initial prototype)
- a sandbox gravitational simulation using arbritary units
- users could click and launch particles towards a central star
- used a simplified, non-physical gravitational constant
- not based on real physics or scales

### solar_system_v1.py
- transitioned to real-world units
- simulated planets orbiting the sun using accurate mass, radius, and initial velocities
- used basic velocity-verlet integration
- added orbit trails, label, and zoom functionality
- introduced revolution tracking and total energy diagnostics

### solar_system_v2.py
- replaced basic integrator with a leapfrog integrator better energy stability
- introduced momentum correction to stabilize the suns's motion
- simplied modularity of force calculations
- allowed planet-specific tracking for orbit completion and energy reporting

### solar_system_v3.py
- upgraded to yoshida 4th-order sympletic integration for long-term orbital accuracy
- barycentric frame implementation: system now centers on the true center of mass
- kinetic energy tracking for all planet, saved to planet_data.csv every 100 days
- real-time kinetic energy plotting after simulation using matplotlib
- enhanced performance and cleaner force computations via compute_all_forces()

