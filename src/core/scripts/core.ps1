$ErrorActionPreference = 'Stop'

if ($args.Count -gt 0 -and $args[0] -eq 'build' -and ($args.Count -eq 1 -or $args[1] -eq 'core')) {
    $rvizDirectory = Join-Path $PSScriptRoot '..\embodied_nexus_core\web_rviz'
    Push-Location $rvizDirectory
    try {
        & npm.cmd ci
        if ($LASTEXITCODE -ne 0) { throw "npm ci failed with exit code $LASTEXITCODE." }
        & npm.cmd run build
        if ($LASTEXITCODE -ne 0) { throw "Web RViz build failed with exit code $LASTEXITCODE." }
    }
    finally { Pop-Location }
}

$scriptWindowsPath = (Resolve-Path (Join-Path $PSScriptRoot 'core.sh')).Path -replace '\\', '/'
$wslDistro = if ($env:EMBODIED_NEXUS_WSL_DISTRO) { $env:EMBODIED_NEXUS_WSL_DISTRO } else { 'Ubuntu-22.04' }
$scriptWslPath = (& wsl.exe -d $wslDistro -- wslpath -u -a $scriptWindowsPath).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($scriptWslPath)) {
    throw 'Could not convert the Core script path to WSL.'
}

$oldWslEnv = $env:WSLENV
$wslEnvEntries = if ($oldWslEnv) { @($oldWslEnv -split ':' | Where-Object { $_ }) } else { @() }
foreach ($name in @('EMBODIED_NEXUS_CORE_TOKEN', 'IGN_IP', 'ROS_DOMAIN_ID')) {
    $found = $false
    foreach ($entry in $wslEnvEntries) {
        if (($entry -split '/')[0] -eq $name) { $found = $true; break }
    }
    if (-not $found) { $wslEnvEntries += "$name/u" }
}

$exitCode = 1
try {
    $env:WSLENV = $wslEnvEntries -join ':'
    $wslArguments = @('-d', $wslDistro, '--', 'bash', $scriptWslPath) + @($args)
    if ($args.Count -gt 0 -and $args[0] -eq 'build' -and ($args.Count -eq 1 -or $args[1] -eq 'core')) {
        $wslArguments += '--frontend-ready'
    }
    & wsl.exe @wslArguments
    $exitCode = $LASTEXITCODE
}
finally {
    $env:WSLENV = $oldWslEnv
}
if ($exitCode -ne 0) { throw "Core WSL script failed with exit code $exitCode." }
