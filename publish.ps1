[CmdletBinding()]
param(
    [string]$Version,
    [string]$OutputDirectory,
    [switch]$IncludeSymbols,
    [switch]$NoArchive
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-FullPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    return [System.IO.Path]::GetFullPath($Path)
}

function Assert-ChildPath {
    param(
        [Parameter(Mandatory = $true)][string]$Parent,
        [Parameter(Mandatory = $true)][string]$Child
    )

    $parentPath = (Get-FullPath $Parent).TrimEnd(
        [System.IO.Path]::DirectorySeparatorChar,
        [System.IO.Path]::AltDirectorySeparatorChar)
    $childPath = Get-FullPath $Child
    $parentPrefix = $parentPath +
        [System.IO.Path]::DirectorySeparatorChar

    if (-not $childPath.StartsWith(
        $parentPrefix,
        [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Unsafe output path: '$childPath' is not under '$parentPath'."
    }
}

function Find-MSBuild {
    $candidates = New-Object System.Collections.Generic.List[string]

    $command = Get-Command MSBuild.exe -ErrorAction SilentlyContinue
    if ($null -ne $command -and -not [string]::IsNullOrWhiteSpace($command.Source)) {
        $candidates.Add($command.Source)
    }

    $programFilesX86 = [Environment]::GetFolderPath(
        [Environment+SpecialFolder]::ProgramFilesX86)
    $vswhere = Join-Path `
        $programFilesX86 `
        "Microsoft Visual Studio\Installer\vswhere.exe"

    if (Test-Path -LiteralPath $vswhere) {
        $located = & $vswhere `
            -latest `
            -products * `
            -requires Microsoft.Component.MSBuild `
            -find "MSBuild\**\Bin\MSBuild.exe"

        foreach ($path in $located) {
            if (-not [string]::IsNullOrWhiteSpace($path)) {
                $candidates.Add($path)
            }
        }
    }

    $visualStudioRoots = @(
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio"),
        (Join-Path $programFilesX86 "Microsoft Visual Studio")
    )

    foreach ($root in $visualStudioRoots) {
        if (-not (Test-Path -LiteralPath $root)) {
            continue
        }

        $found = Get-ChildItem `
            -LiteralPath $root `
            -Filter MSBuild.exe `
            -Recurse `
            -ErrorAction SilentlyContinue |
            Where-Object {
                $_.FullName -match
                    "\\MSBuild\\Current\\Bin(\\amd64)?\\MSBuild\.exe$"
            } |
            Sort-Object FullName -Descending

        foreach ($item in $found) {
            $candidates.Add($item.FullName)
        }
    }

    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (-not (Test-Path -LiteralPath $candidate)) {
            continue
        }

        $versionInfo = [System.Diagnostics.FileVersionInfo]::GetVersionInfo(
            $candidate)
        if ($versionInfo.FileMajorPart -ge 15) {
            return $candidate
        }
    }

    throw @"
Visual Studio MSBuild 15 or newer was not found.
Install Visual Studio or Build Tools with the '.NET desktop build tools' workload.
The legacy C:\Windows\Microsoft.NET MSBuild is not supported by this project.
"@
}

function Remove-PublishOnlyArtifacts {
    param(
        [Parameter(Mandatory = $true)][string]$PackageDirectory,
        [Parameter(Mandatory = $true)][bool]$KeepSymbols
    )

    $files = Get-ChildItem `
        -LiteralPath $PackageDirectory `
        -File `
        -Recurse

    foreach ($file in $files) {
        $remove = $false

        if (-not $KeepSymbols -and $file.Extension -ieq ".pdb") {
            $remove = $true
        }

        if ($file.Extension -ieq ".xml" -or
            $file.Extension -ieq ".log" -or
            $file.Name -like "*Probe.exe" -or
            $file.Name -like "*Probe.exe.config" -or
            $file.Name -like "history_energy.*") {
            $remove = $true
        }

        if ($remove) {
            Assert-ChildPath `
                -Parent $PackageDirectory `
                -Child $file.FullName
            Remove-Item -LiteralPath $file.FullName -Force
        }
    }
}

$repositoryRoot = Get-FullPath $PSScriptRoot
$solutionPath = Join-Path $repositoryRoot "VirtualDriverApp.sln"
$projectDirectory = Join-Path $repositoryRoot "VirtualDriverApp"
$releaseDirectory = Join-Path $projectDirectory "bin\Release"
$applicationPath = Join-Path $releaseDirectory "VirtualDriverApp.exe"

if (-not (Test-Path -LiteralPath $solutionPath)) {
    throw "Solution not found: $solutionPath"
}

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $repositoryRoot "publish"
}
elseif (-not [System.IO.Path]::IsPathRooted($OutputDirectory)) {
    $OutputDirectory = Join-Path $repositoryRoot $OutputDirectory
}

$outputRoot = Get-FullPath $OutputDirectory
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null

$msbuildPath = Find-MSBuild
Write-Host "MSBuild: $msbuildPath"
Write-Host "Building VirtualDriverApp (Release)..."

$buildArguments = @(
    $solutionPath,
    "/restore",
    "/t:Rebuild",
    "/p:Configuration=Release",
    "/p:RestorePackagesConfig=true",
    "/m",
    "/v:minimal",
    "/nologo"
)

& $msbuildPath @buildArguments
if ($LASTEXITCODE -ne 0) {
    throw "Release build failed with exit code $LASTEXITCODE."
}

if (-not (Test-Path -LiteralPath $applicationPath)) {
    throw "Build succeeded but the application was not found: $applicationPath"
}

if ([string]::IsNullOrWhiteSpace($Version)) {
    $Version = [System.Diagnostics.FileVersionInfo]::GetVersionInfo(
        $applicationPath).FileVersion
}

if ([string]::IsNullOrWhiteSpace($Version)) {
    $Version = "0.0.0"
}

$Version = $Version.Trim()
if ($Version -notmatch "^[0-9A-Za-z][0-9A-Za-z._-]*$") {
    throw "Version contains unsupported characters: '$Version'."
}

$packageName = "VirtualDriverApp-$Version-win-net472"
$packageDirectory = Get-FullPath (Join-Path $outputRoot $packageName)
$archivePath = Get-FullPath (Join-Path $outputRoot ($packageName + ".zip"))
$checksumPath = $archivePath + ".sha256"

Assert-ChildPath -Parent $outputRoot -Child $packageDirectory
Assert-ChildPath -Parent $outputRoot -Child $archivePath
Assert-ChildPath -Parent $outputRoot -Child $checksumPath

if (Test-Path -LiteralPath $packageDirectory) {
    Remove-Item -LiteralPath $packageDirectory -Recurse -Force
}

New-Item -ItemType Directory -Path $packageDirectory -Force | Out-Null
Copy-Item `
    -Path (Join-Path $releaseDirectory "*") `
    -Destination $packageDirectory `
    -Recurse `
    -Force

Remove-PublishOnlyArtifacts `
    -PackageDirectory $packageDirectory `
    -KeepSymbols $IncludeSymbols.IsPresent

foreach ($documentationFile in @("README.md", "LICENSE.txt")) {
    $source = Join-Path $repositoryRoot $documentationFile
    if (Test-Path -LiteralPath $source) {
        Copy-Item `
            -LiteralPath $source `
            -Destination $packageDirectory `
            -Force
    }
}

$gitCommit = "unavailable"
$gitState = "unavailable"
$gitCommand = Get-Command git.exe -ErrorAction SilentlyContinue
if ($null -ne $gitCommand) {
    $commitOutput = & $gitCommand.Source `
        -C $repositoryRoot `
        rev-parse --short HEAD 2>$null
    if ($LASTEXITCODE -eq 0 -and $commitOutput) {
        $gitCommit = ($commitOutput | Select-Object -First 1).Trim()
        $statusOutput = & $gitCommand.Source `
            -C $repositoryRoot `
            status --porcelain 2>$null
        $gitState = if ($statusOutput) { "dirty" } else { "clean" }
    }
}

$buildInformation = @(
    "Product: VirtualDriverApp",
    "Version: $Version",
    "Configuration: Release",
    "Target framework: .NET Framework 4.7.2",
    "Built at: $([DateTimeOffset]::Now.ToString('yyyy-MM-ddTHH:mm:ssK'))",
    "Git commit: $gitCommit",
    "Git state: $gitState"
)

$buildInformation | Set-Content `
    -LiteralPath (Join-Path $packageDirectory "BUILD-INFO.txt") `
    -Encoding UTF8

$manifestLines = Get-ChildItem `
    -LiteralPath $packageDirectory `
    -File `
    -Recurse |
    Sort-Object FullName |
    ForEach-Object {
        $relativePath = $_.FullName.Substring(
            $packageDirectory.Length + 1).Replace("\", "/")
        $hash = (Get-FileHash `
            -LiteralPath $_.FullName `
            -Algorithm SHA256).Hash.ToLowerInvariant()
        "$hash  $relativePath"
    }

$manifestLines | Set-Content `
    -LiteralPath (Join-Path $packageDirectory "manifest-sha256.txt") `
    -Encoding ASCII

$archiveResult = "not requested"
$checksumResult = "not requested"

if (-not $NoArchive.IsPresent) {
    if (Test-Path -LiteralPath $archivePath) {
        Remove-Item -LiteralPath $archivePath -Force
    }

    if (Test-Path -LiteralPath $checksumPath) {
        Remove-Item -LiteralPath $checksumPath -Force
    }

    Compress-Archive `
        -LiteralPath $packageDirectory `
        -DestinationPath $archivePath `
        -CompressionLevel Optimal

    $archiveHash = (Get-FileHash `
        -LiteralPath $archivePath `
        -Algorithm SHA256).Hash.ToLowerInvariant()
    "$archiveHash  $([System.IO.Path]::GetFileName($archivePath))" |
        Set-Content `
            -LiteralPath $checksumPath `
            -Encoding ASCII

    $archiveResult = $archivePath
    $checksumResult = $checksumPath
}

Write-Host "Publish completed."
[PSCustomObject]@{
    Version = $Version
    PackageDirectory = $packageDirectory
    Archive = $archiveResult
    ArchiveChecksum = $checksumResult
    SymbolsIncluded = $IncludeSymbols.IsPresent
}
