$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$frameDir = Join-Path $root "matlab\webcam_frames"
$pidFile = Join-Path $frameDir "camera_stream.pid"
$cropPidFile = Join-Path $frameDir "crop_stream.pid"
$logFile = Join-Path $frameDir "camera_stream.log"
$cropLogFile = Join-Path $frameDir "crop_stream.log"
$latestFile = Join-Path $frameDir "latest_board.jpg"
$cropFile = Join-Path $frameDir "latest_board_crop.jpg"
$ffmpeg = "C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS Flow Simulation\binCFW\ffmpeg.exe"
$python = "C:\Users\Tiger\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"

if (!(Test-Path -LiteralPath $frameDir)) {
    New-Item -ItemType Directory -Force -Path $frameDir | Out-Null
}

if (Test-Path -LiteralPath $pidFile) {
    $oldPid = Get-Content -LiteralPath $pidFile -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($oldPid) {
        $oldProcess = Get-Process -Id ([int]$oldPid) -ErrorAction SilentlyContinue
        if ($oldProcess) {
            Write-Host "Camera stream already running. PID: $oldPid"
            exit 0
        }
    }
}

if (!(Test-Path -LiteralPath $ffmpeg)) {
    throw "ffmpeg not found: $ffmpeg"
}

$arguments = @(
    "-hide_banner",
    "-nostats",
    "-loglevel", "error",
    "-y",
    "-f", "dshow",
    "-i", "video=OsmoAction4",
    "-vf", "fps=2",
    "-update", "1",
    "-q:v", "2",
    $latestFile
)

$argumentString = ($arguments | ForEach-Object {
    if ($_ -match '[\s"]') {
        '"' + ($_ -replace '"', '\"') + '"'
    } else {
        $_
    }
}) -join " "

$process = Start-Process -FilePath $ffmpeg `
    -ArgumentList $argumentString `
    -WorkingDirectory $root `
    -WindowStyle Hidden `
    -PassThru

Start-Sleep -Milliseconds 800
$process.Refresh()
if ($process.HasExited) {
    throw "ffmpeg camera stream exited immediately. Check camera availability or permissions."
}

Set-Content -LiteralPath $pidFile -Value $process.Id -Encoding ASCII
Set-Content -LiteralPath $logFile -Value "Camera stream started PID=$($process.Id) at $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -Encoding UTF8

$cropArguments = @(
    (Join-Path $root "cropBoardStream.py"),
    $latestFile,
    $cropFile,
    "285",
    "220",
    "1000",
    "705"
)

$cropArgumentString = ($cropArguments | ForEach-Object {
    if ($_ -match '[\s"]') {
        '"' + ($_ -replace '"', '\"') + '"'
    } else {
        $_
    }
}) -join " "

$cropProcess = Start-Process -FilePath $python `
    -ArgumentList $cropArgumentString `
    -WorkingDirectory $root `
    -WindowStyle Hidden `
    -PassThru

Start-Sleep -Milliseconds 300
$cropProcess.Refresh()
if ($cropProcess.HasExited) {
    Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $pidFile -Force -ErrorAction SilentlyContinue
    throw "crop stream exited immediately."
}

Set-Content -LiteralPath $cropPidFile -Value $cropProcess.Id -Encoding ASCII
Set-Content -LiteralPath $cropLogFile -Value "Crop stream started PID=$($cropProcess.Id) at $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -Encoding UTF8

Write-Host "Camera stream started. PID: $($process.Id)"
Write-Host "Crop stream started. PID: $($cropProcess.Id)"
Write-Host "Latest frame: $latestFile"
Write-Host "Latest crop: $cropFile"
