# Starts the simulation in Docker on Windows (Docker Desktop).
#
#   .\run.ps1            desktop in the browser at http://localhost:6080/vnc.html
#   .\run.ps1 headless   no windows, records bagfiles\$env:BAG_NAME.bag for $env:DURATION seconds
#
# Inside WSL2 use ./run.sh, which opens the windows on the desktop through WSLg.
param([ValidateSet("web", "headless")][string]$Mode = "web")

Set-Location -Path $PSScriptRoot

if ($Mode -eq "web") {
    Write-Host "Desktop in the browser at http://localhost:6080/vnc.html (software rendering)"
    docker compose up web
} else {
    Write-Host "Headless run on a virtual display (software rendering)"
    docker compose run --rm headless
}
