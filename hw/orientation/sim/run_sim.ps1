# PowerShell version.  Run:  powershell -ExecutionPolicy Bypass -File hw\orientation\sim\run_sim.ps1
# (extra arguments are passed to gen_vectors.py, e.g.  --sectors 32 --rounding nearest)
$ErrorActionPreference = "Stop"
$msys = "C:\msys64\ucrt64\bin"            # python + iverilog/vvp installed with pacman
if (Test-Path $msys) { $env:Path = "$msys;$env:Path" }
Set-Location $PSScriptRoot                # the testbench reads img.hex etc. from here
python gen_vectors.py @args
iverilog -g2012 -Wall -o sim_orient -c files.f
vvp sim_orient
vvp sim_orient +gap=40
