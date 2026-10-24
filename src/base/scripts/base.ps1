$ErrorActionPreference = 'Stop'
$scriptWindowsPath = (Resolve-Path (Join-Path $PSScriptRoot 'base.sh')).Path -replace '\\', '/'
$wslDistro = if ($env:EMBODIED_NEXUS_WSL_DISTRO) { $env:EMBODIED_NEXUS_WSL_DISTRO } else { 'Ubuntu-22.04' }
$scriptWslPath = (& wsl.exe -d $wslDistro -- wslpath -u -a $scriptWindowsPath).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($scriptWslPath)) { throw 'Could not convert the base script path to WSL.' }
$oldWslEnv = $env:WSLENV
$wslEnvEntries = if ($oldWslEnv) { @($oldWslEnv -split ':' | Where-Object { $_ }) } else { @() }
foreach ($name in @('EMBODIED_NEXUS_CORE_TOKEN', 'IGN_IP', 'ROS_DOMAIN_ID')) {
    $found = $false
    foreach ($entry in $wslEnvEntries) { if (($entry -split '/')[0] -eq $name) { $found = $true; break } }
    if (-not $found) { $wslEnvEntries += "$name/u" }
}
$exitCode = 1
try {
    $env:WSLENV = $wslEnvEntries -join ':'
    & wsl.exe -d $wslDistro -- bash $scriptWslPath @args
    $exitCode = $LASTEXITCODE
} finally { $env:WSLENV = $oldWslEnv }
if ($exitCode -ne 0) { throw "Base WSL script failed with exit code $exitCode." }
