[CmdletBinding()]
param(
    [ValidateSet("windows", "linux")]
    [string]$TargetOS = "windows",

    [ValidateSet("amd64", "arm64")]
    [string]$TargetArch = "amd64",

    [string]$Version = "1.0.0",

    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$projectDirectory = $PSScriptRoot
$publishDirectory = Join-Path $projectDirectory "publish\$TargetOS-$TargetArch"
$executableName = if ($TargetOS -eq "windows") { "modbus-flow-adapter.exe" } else { "modbus-flow-adapter" }
$executablePath = Join-Path $publishDirectory $executableName

Push-Location $projectDirectory
$previousGOOS = $env:GOOS
$previousGOARCH = $env:GOARCH
try {
    Write-Host "[1/4] 下载/检查 Go 依赖..." -ForegroundColor Cyan
    go mod download

    if (-not $SkipTests) {
        Write-Host "[2/4] 运行自动测试..." -ForegroundColor Cyan
        go test ./...
    }
    else {
        Write-Host "[2/4] 已跳过自动测试。" -ForegroundColor Yellow
    }

    Write-Host "[3/4] 构建 $TargetOS/$TargetArch..." -ForegroundColor Cyan
    New-Item -ItemType Directory -Force -Path $publishDirectory | Out-Null
    $env:GOOS = $TargetOS
    $env:GOARCH = $TargetArch
    go build -trimpath -ldflags "-s -w -X main.version=$Version" -o $executablePath ./cmd/flow-adapter

    Write-Host "[4/4] 复制运行配置和说明..." -ForegroundColor Cyan
    Copy-Item -Force -LiteralPath (Join-Path $projectDirectory "config.yaml") -Destination $publishDirectory
    Copy-Item -Force -LiteralPath (Join-Path $projectDirectory "README.md") -Destination $publishDirectory

    Write-Host "发布完成: $publishDirectory" -ForegroundColor Green
    Write-Host "请先修改 config.yaml，再运行 $executableName"
}
finally {
    $env:GOOS = $previousGOOS
    $env:GOARCH = $previousGOARCH
    Pop-Location
}
