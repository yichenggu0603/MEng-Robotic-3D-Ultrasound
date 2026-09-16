# Controller Versions

| File | Role in development |
|---|---|
| `impedance_control_v1.cpp` | Initial Cartesian impedance, force-bias calibration, Z-equilibrium adjustment, slow X scan, and joint-6 locking |
| `impedance_control_v2.cpp` | Reduced stiffness, removed joint-6 locking, and introduced manual XY guidance and orientation following |
| `impedance_control_v3.cpp` | Added direction-based XY velocity guidance, Z contact regulation, velocity damping, and task-force limits |
| `impedance_control_v4.cpp` | Tuned interaction thresholds, limits, direction parameters, and Z response |
| `impedance_control_v5.cpp` | Increased XY reference speed, reduced rotational resistance, and scaled joint-6 task torque |
| `impedance_control_v5.1.cpp` | Added buffered approximately 100 Hz logging, timestamped filenames, and controlled Enter-key termination |

These files contain C++ and are intended for integration into the libfranka
examples build. The original project archive used historical `.c` extensions;
the repository uses `.cpp` so GitHub and development tools identify the language
correctly. Version v5.1 was used for the reported experiments.

The controllers are research code and have not been validated for clinical use.
