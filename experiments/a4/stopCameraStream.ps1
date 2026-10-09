$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$frameDir = Join-Path $root "matlab\webcam_frames"
$pidFile = Join-Path $frameDir "camera_stream.pid"
$cropPidFile = Join-Path $frameDir "crop_stream.pid"

function Stop-ByPidFile([string]$Path, [string]$Label) {
    if (!(Test-Path -LiteralPath $Path)) {
        Write-Host "No $Label PID file found."
        return
    }

    $pidText = Get-Content -LiteralPath $Path -ErrorAction SilentlyContinue | Select-Object -First 1
    if (!$pidText) {
        Remove-Item -LiteralPath $Path -Force
        Write-Host "$Label PID file was empty; removed it."
        return
    }

    $process = Get-Process -Id ([int]$pidText) -ErrorAction SilentlyContinue
    if ($process) {
        Stop-Process -Id ([int]$pidText) -Force
        Write-Host "Stopped $Label PID: $pidText"
    } else {
        Write-Host "$Label process was not running. PID: $pidText"
    }

    Remove-Item -LiteralPath $Path -Force
}

Stop-ByPidFile -Path $pidFile -Label "camera stream"
Stop-ByPidFile -Path $cropPidFile -Label "crop stream"
