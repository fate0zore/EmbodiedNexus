$ErrorActionPreference = 'Stop'
$distro = if ($env:EMBODIED_NEXUS_WSL_DISTRO) { $env:EMBODIED_NEXUS_WSL_DISTRO } else { 'Ubuntu-22.04' }
$scriptPath = (Resolve-Path (Join-Path $PSScriptRoot 'build.sh')).Path -replace '\\', '/'
$linuxPath = (& wsl.exe -d $distro -- wslpath -u -a $scriptPath).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxPath) { throw 'Unable to resolve the WSL build script path.' }
& wsl.exe -d $distro -- bash $linuxPath @args
exit $LASTEXITCODE
