$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = "C:\Users\Tiger\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$latest = Join-Path $root "matlab\webcam_frames\latest_board.jpg"
$crop = Join-Path $root "matlab\webcam_frames\latest_board_crop.jpg"

if (!(Test-Path -LiteralPath $latest)) {
    throw "Latest board frame not found: $latest"
}

& $python -c "from PIL import Image; im=Image.open(r'$latest'); im.crop((560,175,980,510)).save(r'$crop', quality=95)"
Write-Host "Updated crop: $crop"
