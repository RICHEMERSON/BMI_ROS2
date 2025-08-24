# IMT Asynchronous Hold-on (Windows-friendly)

This scenario mirrors `semi_asynchrounous_2d_refit_interception` but drives the cursor via BMI decoding instead of mouse clicks.

## Components
- observation_layer.py: launches Blackrock passive IR reader
- launch.py: ROS 2 launch for integrator/trainer/predictor (+ buffers)
- main.py: convenience runner that starts observation, decoding, and a presenter
- controller.py: attaches to shared-memory control lists if created elsewhere
- launch_code.ps1: optional PowerShell script with separate steps

Logs are under `log/<timestamp>_imt_hold_on/`.

## Prereqs
- Build done: install populated (ros2 run works)
- PowerShell sourced: `. install\setup.ps1`

## Quick start
- Run everything:
  - `python main.py`

- Or step-by-step (PowerShell):
  - `python observation_layer.py`
  - `ros2 launch .\launch.py system:=0 group:=0 state:=0 algorithm:=refit_kf_lcy_2d`
  - `ros2 run state_reader psychopy_2D_interception`

Adjust `system/group/state` consistently across your other scripts if needed.
