param(
    [switch]$BuildOnly
)

$ErrorActionPreference = 'Stop'

$buildScript = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\tools\build.ps1')
)
$arguments = @{
    Target = 'OfflineTests'
}
if(-not $BuildOnly){
    $arguments.Run = $true
}

& $buildScript @arguments
exit $LASTEXITCODE
