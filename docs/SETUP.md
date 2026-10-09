# Setup and Hardware Configuration

## Requirements

- MATLAB; the original setup notes target R2022b.
- Robotics System Toolbox for `importrobot`, rigid-body models and IK workflows.
- Symbolic Math Toolbox for symbolic Jacobian scripts.
- Arduino IDE, compatible board, servos and joystick for physical control.
- The course's `UoM-RS-Starter_Code`, `UoM_RS_MATLAB_LIBRARIES` and `UoM-RS-Arduino_Project_Library`, obtained separately through the course or their authorized upstream sources.
- Camera scripts additionally depend on Python/Pillow and FFmpeg; the JavaScript camera server needs Node.js.

## Starting with simulation

Open MATLAB in the individual project directory so relative URDF paths resolve. Inspect the selected script's settings, then run a simulation/workspace script such as `projects/01-kinematics/WorkSpace.m` or the Assignment 3 snapshot. Avoid adding every experiment directory to the MATLAB path: duplicate function names can select the wrong implementation.

`projects/03-path-planning/so101.urdf` references `assets/*.stl`, but those meshes were not found in the supplied directory. That model needs its original mesh assets before it can be rendered correctly.

## Preparing hardware experiments

1. Choose one A4 variant and work from that directory.
2. Supply its `external/UoM-RS-Starter_Code`, `external/UoM_RS_MATLAB_LIBRARIES`, and `external/UoM-RS-Arduino_Project_Library` directories. These third-party libraries are intentionally not bundled here.
3. Inspect `setup_a4.m` and run it to configure paths/constants. Other prototype scripts may require the MATLAB library on the path explicitly.
4. Recreate the servo zero-offset calibration and required caches using the chosen variant's calibration/precompute functions. Not every variant contains every generator; inspect the cache references before running a controller.
5. Review COM port, motor IDs/directions, joint limits, board geometry, gripper settings and startup motion for your physical setup.

The camera PowerShell scripts preserve the original machine-specific Python/FFmpeg installation paths and camera device name. Edit those settings before use. The source code has not been executed against MATLAB, Arduino or a physical robot during repository preparation.

## Repository scope

Excluded: ZIP bundles, lectures and assignment handouts, reports/Word documents, third-party CAD, raw audio/video (short curated demos are included under `media/`), MATLAB autosaves, camera captures, runtime logs, servo calibration and generated MAT caches. Small `UoM-RS-Var_Def.mat` constants files are retained for older scripts that load them directly.

Source provenance and redistribution rights have not been established for every coursework/helper file. No open-source license has been assigned. Review course rules and team ownership before making the repository public; see `docs/PUBLISHING.md`.
