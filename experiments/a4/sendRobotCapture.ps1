param(
    [Parameter(Mandatory = $true)]
    [string]$From,

    [Parameter(Mandatory = $true)]
    [string]$Target
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$commandFile = Join-Path $root "matlab\robot_command.txt"
$id = Get-Date -Format "yyyyMMddHHmmssfff"
$command = "CAPTURE,$id,$($From.ToUpper()),$($Target.ToUpper())"
Set-Content -LiteralPath $commandFile -Value $command -Encoding ASCII
Write-Host "Sent robot command: $command"
