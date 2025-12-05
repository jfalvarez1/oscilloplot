# Setup script for Oscilloplot C++ external dependencies
# Run this script from the project root directory

$ErrorActionPreference = "Stop"

Write-Host "Setting up external dependencies for Oscilloplot..." -ForegroundColor Cyan

# Create external directory if it doesn't exist
$externalDir = "external"
if (-not (Test-Path $externalDir)) {
    New-Item -ItemType Directory -Path $externalDir | Out-Null
}

Set-Location $externalDir

# Clone Dear ImGui
Write-Host "`nCloning Dear ImGui..." -ForegroundColor Yellow
if (-not (Test-Path "imgui")) {
    git clone --depth 1 --branch v1.91.6 https://github.com/ocornut/imgui.git
} else {
    Write-Host "ImGui already exists, skipping..." -ForegroundColor Green
}

# Clone ImPlot
Write-Host "`nCloning ImPlot..." -ForegroundColor Yellow
if (-not (Test-Path "implot")) {
    git clone --depth 1 --branch v0.16 https://github.com/epezent/implot.git
} else {
    Write-Host "ImPlot already exists, skipping..." -ForegroundColor Green
}

# Clone PocketFFT (header-only)
Write-Host "`nCloning PocketFFT..." -ForegroundColor Yellow
if (-not (Test-Path "pocketfft")) {
    git clone --depth 1 https://github.com/mreineck/pocketfft.git
} else {
    Write-Host "PocketFFT already exists, skipping..." -ForegroundColor Green
}

Set-Location ..

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "External dependencies setup complete!" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Cyan

Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Install vcpkg if not already installed:"
Write-Host "   git clone https://github.com/Microsoft/vcpkg.git"
Write-Host "   cd vcpkg && bootstrap-vcpkg.bat"
Write-Host ""
Write-Host "2. Install dependencies via vcpkg:"
Write-Host "   vcpkg install sdl2 portaudio libsndfile glm --triplet=x64-windows"
Write-Host ""
Write-Host "3. Configure and build:"
Write-Host "   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake"
Write-Host "   cmake --build build --config Release"
Write-Host ""
