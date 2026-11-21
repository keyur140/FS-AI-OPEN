# fsai-open: Windows (PowerShell + Docker Desktop) version of install.sh / run.sh
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
# Usage:  .\run.ps1 install   then   .\run.ps1 demo [event] [seed]  |  .\run.ps1 test  |  .\run.ps1 verify
param([string]$Command = "help", [string]$Event = "autocross", [string]$Seed = "0", [string]$Clutter = "0")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$Image = "fsai-open:0.2.0"
$Here = (Get-Location).Path
switch ($Command) {
  "install" { docker build -f docker/Dockerfile -t $Image . }
  "demo"    { New-Item -ItemType Directory -Force local_runs/runs/ros2_autonomy | Out-Null
              docker run --rm -v "${Here}/local_runs/runs/ros2_autonomy:/out" $Image sim event:=$Event seed:=$Seed clutter:="${Clutter}.0" result:=/out/result_${Event}_${Seed}.json replay:=/out/replay_${Event}_${Seed}.html
              Write-Host "Replay (open in any browser): local_runs/runs/ros2_autonomy/replay_${Event}_${Seed}.html" }
  "bridge-demo" { New-Item -ItemType Directory -Force local_runs/runs/ros2_demo | Out-Null
              docker run --rm -v "${Here}/local_runs/runs/ros2_demo:/out" $Image demo result:=/out/result.json }
  "test"    { New-Item -ItemType Directory -Force local_runs/runs, local_runs/reports | Out-Null
              docker run --rm -v "${Here}/local_runs/runs:/opt/fsai-open/runs" -v "${Here}/local_runs/reports:/opt/fsai-open/reports" $Image test --no-hash }
  "verify"  { docker run --rm -v "${Here}:/check:ro" -w /check -e PYTHONPATH=/check $Image python3 /check/verify.py }
  default   { Write-Host "Usage: .\run.ps1 install | demo [event] [seed] | bridge-demo | test | verify   (the real car needs Linux: see docs/TEAM_SETUP_GUIDE.md)" }
}
