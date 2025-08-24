# PowerShell helper to start the IMT asynchronous hold-on session
# Ensure ROS 2 env is sourced for PowerShell before running this (install\setup.ps1)

$timestamp = [int](Get-Date -UFormat %s) * 10
$logDir = Join-Path (Join-Path (Get-Location) 'log') ("{0}_imt_hold_on" -f $timestamp)
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

# Observation
python .\observation_layer.py 2> (Join-Path $logDir 'observation.log')

# Decoding (launch integrator/trainer/predictor/buffers)
ros2 launch .\launch.py system:=0 group:=0 state:=0 algorithm:=refit_kf_lcy_2d 2> (Join-Path $logDir 'decoding_element.log')

# Presenter using BMI service
ros2 run state_reader psychopy_2D_interception 2> (Join-Path $logDir 'behavior.log')
