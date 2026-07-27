param(
    [ValidateSet('Main', 'GUI', 'DeltaServer', 'LegacyTest', 'OfflineTests')]
    [string]$Target = 'Main',

    [string]$OutputDirectory,

    [string]$VsDevCmd,

    [switch]$Run
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Get-FullPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$BasePath
    )

    if([System.IO.Path]::IsPathRooted($Path)){
        return [System.IO.Path]::GetFullPath($Path)
    }

    return [System.IO.Path]::GetFullPath((Join-Path $BasePath $Path))
}

function Assert-FileExists {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    if(-not (Test-Path -LiteralPath $Path -PathType Leaf)){
        throw "$Description was not found: $Path"
    }
}

function Resolve-VsDevCmd {
    param(
        [string]$ExplicitPath
    )

    if($ExplicitPath){
        $resolvedExplicitPath = [System.IO.Path]::GetFullPath($ExplicitPath)
        Assert-FileExists -Path $resolvedExplicitPath `
            -Description 'The requested Visual Studio developer command script'
        return $resolvedExplicitPath
    }

    if($env:CU_BRICK_VSDEVCMD){
        $resolvedEnvironmentPath =
            [System.IO.Path]::GetFullPath($env:CU_BRICK_VSDEVCMD)
        Assert-FileExists -Path $resolvedEnvironmentPath `
            -Description 'CU_BRICK_VSDEVCMD'
        return $resolvedEnvironmentPath
    }

    if($env:VSINSTALLDIR){
        $developerCommand = Join-Path $env:VSINSTALLDIR `
            'Common7\Tools\VsDevCmd.bat'
        if(Test-Path -LiteralPath $developerCommand -PathType Leaf){
            return [System.IO.Path]::GetFullPath($developerCommand)
        }
    }

    $vswhereCommand = Get-Command 'vswhere.exe' -ErrorAction SilentlyContinue
    $vswherePath = $null
    if($vswhereCommand){
        $vswherePath = $vswhereCommand.Source
    }elseif(${env:ProgramFiles(x86)}){
        $installerVswhere = Join-Path ${env:ProgramFiles(x86)} `
            'Microsoft Visual Studio\Installer\vswhere.exe'
        if(Test-Path -LiteralPath $installerVswhere -PathType Leaf){
            $vswherePath = $installerVswhere
        }
    }

    if($vswherePath){
        $installationPath = & $vswherePath `
            -latest `
            -products '*' `
            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
            -property installationPath |
            Select-Object -First 1

        if($installationPath){
            $developerCommand = Join-Path $installationPath `
                'Common7\Tools\VsDevCmd.bat'
            if(Test-Path -LiteralPath $developerCommand -PathType Leaf){
                return [System.IO.Path]::GetFullPath($developerCommand)
            }
        }
    }

    throw @'
Visual Studio C++ build tools were not found. Install the "Desktop
development with C++" workload, or provide VsDevCmd.bat through
-VsDevCmd or the CU_BRICK_VSDEVCMD environment variable.
'@
}

function Quote-CommandArgument {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Value
    )

    if($Value.Contains('"')){
        throw "A build path contains an unsupported quote character: $Value"
    }

    return '"' + $Value + '"'
}

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resolvedVsDevCmd = Resolve-VsDevCmd -ExplicitPath $VsDevCmd

if($Run -and $Target -ne 'OfflineTests'){
    throw '-Run is only supported for OfflineTests to prevent accidental hardware access.'
}

$sources = @()
$libraries = @()
$includeDirectories = @()
$compilerOptions = @('/nologo', '/EHsc', '/std:c++14', '/W4', '/permissive-')
$executableName = ''

switch($Target){
    'Main' {
        $executableName = 'main.exe'
        $compilerOptions += @('/Zi', '/MP', '/FS')
        $sources = @(
            'src\BrakeController.cpp',
            'src\CableKinematics.cpp',
            'src\CableController.cpp',
            'src\COMPortNode.cpp',
            'src\DeltaClient.cpp',
            'src\GripperController.cpp',
            'src\Logger.cpp',
            'src\Main.cpp',
            'src\RailController.cpp',
            'src\Robot.cpp',
            'src\TeknicNode.cpp',
            'src\TrajectoryGenerator.cpp',
            'src\TwincatADSNode.cpp',
            'src\utils.cpp'
        )
        $libraries = @(
            'Dependencies\sFoundation20\win32\sFoundation20.lib',
            'Dependencies\TcAdsDll\Lib\TcAdsDll.lib'
        )
    }
    'GUI' {
        $executableName = 'GUI.exe'
        $compilerOptions += @('/Zi', '/MD', '/D_UNICODE', '/DWXUSINGDLL')
        $sources = @('src\GUI.cpp')
        $includeDirectories = @(
            'Dependencies\wxMSW\include',
            'Dependencies\wxMSW\lib\vc14x_dll\mswud'
        )
        $libraries = @(
            'Dependencies\sFoundation20\win32\sFoundation20.lib',
            'Dependencies\TcAdsDll\Lib\TcAdsDll.lib',
            'Dependencies\wxMSW\lib\vc14x_dll\wxbase32u.lib',
            'Dependencies\wxMSW\lib\vc14x_dll\wxmsw32u_core.lib'
        )
    }
    'DeltaServer' {
        $executableName = 'DeltaServer.exe'
        $compilerOptions += @('/Zi', '/MD', '/D_UNICODE', '/DWXUSINGDLL')
        $sources = @(
            'src\DeltaServer.cpp',
            'src\RailController.cpp',
            'src\TwincatADSNode.cpp'
        )
        $libraries = @(
            'Dependencies\sFoundation20\win32\sFoundation20.lib',
            'Dependencies\TcAdsDll\Lib\TcAdsDll.lib'
        )
    }
    'LegacyTest' {
        $executableName = 'TeknicLegacyTest.exe'
        $compilerOptions += @('/Zi')
        $sources = @(
            'test\test.cpp',
            'test\TeknicNode.cpp',
            'test\Logger.cpp'
        )
        $libraries = @(
            'Dependencies\sFoundation20\win32\sFoundation20.lib',
            'Dependencies\TcAdsDll\Lib\TcAdsDll.lib'
        )
    }
    'OfflineTests' {
        $executableName = 'offline_tests.exe'
        $sources = @(
            'test\offline\OfflineTests.cpp',
            'src\BrakeController.cpp',
            'src\CableKinematics.cpp',
            'src\COMPortNode.cpp',
            'src\GripperController.cpp',
            'src\TrajectoryGenerator.cpp'
        )
    }
}

if($OutputDirectory){
    $outputRoot = Get-FullPath -Path $OutputDirectory -BasePath $repoRoot
    $objectDirectory = Join-Path $outputRoot ('obj\' + $Target)
}elseif($Target -eq 'OfflineTests'){
    $outputRoot = Join-Path ([System.IO.Path]::GetTempPath()) `
        'cu_brick_offline_tests'
    $objectDirectory = Join-Path $outputRoot 'obj'
}else{
    $outputRoot = Join-Path $repoRoot 'bin'
    $objectDirectory = Join-Path $repoRoot ('temp\' + $Target)
}

New-Item -ItemType Directory -Force -Path $outputRoot, $objectDirectory |
    Out-Null

$resolvedSources = @(
    $sources | ForEach-Object {
        $path = Get-FullPath -Path $_ -BasePath $repoRoot
        Assert-FileExists -Path $path -Description 'Source file'
        $path
    }
)
$resolvedLibraries = @(
    $libraries | ForEach-Object {
        $path = Get-FullPath -Path $_ -BasePath $repoRoot
        Assert-FileExists -Path $path -Description 'Library'
        $path
    }
)
$resolvedIncludes = @(
    $includeDirectories | ForEach-Object {
        $path = Get-FullPath -Path $_ -BasePath $repoRoot
        if(-not (Test-Path -LiteralPath $path -PathType Container)){
            throw "Include directory was not found: $path"
        }
        $path
    }
)

$executablePath = Join-Path $outputRoot $executableName
$compilerPdbPath = Join-Path $objectDirectory ($Target + '-compiler.pdb')

$compilerArguments = @($compilerOptions)
$compilerArguments += '/Fe' + (Quote-CommandArgument -Value $executablePath)
$compilerArguments += '/Fd' + (Quote-CommandArgument -Value $compilerPdbPath)
$compilerArguments += @(
    $resolvedIncludes | ForEach-Object {
        '/I' + (Quote-CommandArgument -Value $_)
    }
)
$compilerArguments += @(
    $resolvedSources | ForEach-Object {
        Quote-CommandArgument -Value $_
    }
)
$compilerArguments += @(
    $resolvedLibraries | ForEach-Object {
        Quote-CommandArgument -Value $_
    }
)

$compileCommand = 'cl.exe ' + ($compilerArguments -join ' ')
$developerCommand =
    'call ' + (Quote-CommandArgument -Value $resolvedVsDevCmd) +
    ' -arch=x86 >nul && ' + $compileCommand
$commandProcessor = if($env:ComSpec){ $env:ComSpec }else{ 'cmd.exe' }

Write-Host "Building $Target for x86..."
Push-Location $objectDirectory
try{
    & $commandProcessor /d /s /c $developerCommand
    if($LASTEXITCODE -ne 0){
        exit $LASTEXITCODE
    }
}finally{
    Pop-Location
}

Write-Host "Built ${Target}: $executablePath"

if($Run){
    & $executablePath
    exit $LASTEXITCODE
}
