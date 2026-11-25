[CmdletBinding()]
param(
    [ValidateRange(1, 3650)]
    [int]$OlderThanDays = 7,

    [switch]$Apply
)

$ErrorActionPreference = "Stop"

function Invoke-Docker {
    param(
        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$DockerArguments
    )

    & docker @DockerArguments
    if ($LASTEXITCODE -ne 0) {
        throw "docker $($DockerArguments -join ' ') failed with exit code $LASTEXITCODE"
    }
}

$serverOs = & docker version --format '{{.Server.Os}}' 2>$null
if ($LASTEXITCODE -ne 0 -or -not $serverOs) {
    throw "Docker Desktop's Linux engine is not running. Start Docker Desktop first."
}

Write-Host "Docker usage before cleanup:"
Invoke-Docker system df

if (-not $Apply) {
    Write-Host ""
    Write-Host "Report mode only; nothing was deleted."
    Write-Host "This usage summary is not an exact preview of prune candidates."
    Write-Host "The cleanup preserves all named images, containers, and volumes."
    Write-Host "It removes only dangling images and dangling build cache older than $OlderThanDays days."
    Write-Host "Both prune operations are global to Docker Desktop, so dangling data from other projects may also be removed."
    Write-Host ""
    Write-Host "To apply:"
    Write-Host "powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\docker-cleanup.ps1 -Apply -OlderThanDays $OlderThanDays"
    exit 0
}

$olderThanHours = $OlderThanDays * 24
$ageFilter = "until=${olderThanHours}h"

$containerId = & docker ps `
    --filter "name=^/rashko_ros2_windows$" `
    --format '{{.ID}}' 2>$null
if ($LASTEXITCODE -ne 0) {
    throw "Could not inspect the RAshko container before cleanup."
}
if ($containerId) {
    $activeProcesses = & docker top $containerId -eo args 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw "Could not inspect processes in the running RAshko container."
    }
    $rosPattern = 'ros2 launch|udp_servo_bridge|trajectory_executor|move_group|arm_gui|camera_node|sim_servo_node|robot_state_publisher'
    if ($activeProcesses | Select-String -Pattern $rosPattern -Quiet) {
        throw (
            "Cleanup refused while RAshko ROS processes are active. " +
            "E-stop the arm, turn the servo PSU off, and stop the Hardware/GUI terminals first."
        )
    }
}

Write-Warning "Deleting dangling images and dangling Docker build cache matching '$ageFilter'."
Write-Host "Named images (including rashko-ros2_arm and ROS bases), containers, and volumes are preserved."
Write-Host "A later image rebuild may need internet access to download pruned build dependencies again."

# Without --all, image prune is limited to dangling images. Do not add volume
# pruning here: the ROS build/install/log volumes and unrelated project data
# must survive routine cleanup.
Invoke-Docker image prune --force --filter $ageFilter

# Docker BuildKit cache is not reliably project-labelled. Limit deletion by
# both age and dangling status, and clearly report that it is global.
Invoke-Docker builder prune --force --filter $ageFilter

Write-Host ""
Write-Host "Docker usage after cleanup:"
Invoke-Docker system df
