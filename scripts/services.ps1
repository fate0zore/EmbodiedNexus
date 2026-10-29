$ErrorActionPreference = 'Stop'
$distro = if ($env:EMBODIED_NEXUS_WSL_DISTRO) { $env:EMBODIED_NEXUS_WSL_DISTRO } else { 'Ubuntu-22.04' }
$path = (Resolve-Path (Join-Path $PSScriptRoot 'services.sh')).Path -replace '\\', '/'
$linuxPath = (& wsl.exe -d $distro -- wslpath -u -a $path).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxPath) { throw 'Unable to resolve the WSL script path.' }
$previous = $env:WSLENV
try {
    $entries = @($previous -split ':' | Where-Object { $_ })
    foreach ($name in @('EMBODIED_NEXUS_CORE_TOKEN', 'EMBODIED_NEXUS_SETUP', 'IGN_IP', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'LIBGL_ALWAYS_SOFTWARE', 'GALLIUM_DRIVER')) {
        $value = [Environment]::GetEnvironmentVariable($name, 'Process')
        if ($value -and -not ($entries | Where-Object { ($_ -split '/')[0] -eq $name })) { $entries += "$name/u" }
    }
    $env:WSLENV = $entries -join ':'
    & wsl.exe -d $distro -- bash $linuxPath @args
    $result = $LASTEXITCODE
} finally { $env:WSLENV = $previous }
exit $result
