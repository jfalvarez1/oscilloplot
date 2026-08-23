# Builds the standalone Windows release and packages it as
# oscilloplot-v<version>-windows-x64.zip in dist/.
#
# Usage:  ./make_release.ps1 [-SkipBuild]
param([switch]$SkipBuild)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

# Version is owned by CMakeLists.txt
$cmake = Get-Content "$root\CMakeLists.txt" -Raw
if ($cmake -notmatch 'project\(oscilloplot VERSION ([0-9.]+)') {
    throw "Could not read version from CMakeLists.txt"
}
$version = $Matches[1]
$buildDir = "$root\build-standalone"
$stage = "$root\dist\oscilloplot-v$version-windows-x64"

if (-not $SkipBuild) {
    if (-not (Test-Path "$buildDir\CMakeCache.txt")) {
        cmake -S $root -B $buildDir -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        if ($LASTEXITCODE -ne 0) { throw "configure failed" }
    }
    cmake --build $buildDir --config Release --target oscilloplot
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
}

$exe = "$buildDir\Release\oscilloplot.exe"
if (-not (Test-Path $exe)) { throw "missing $exe - build first" }

Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force "$stage\docs" | Out-Null
Copy-Item $exe $stage
Copy-Item "$root\README.md" $stage
Copy-Item "$root\LICENSE" $stage
Copy-Item "$root\docs\user_manual.html" "$stage\docs"

$zip = "$root\dist\oscilloplot-v$version-windows-x64.zip"
Remove-Item $zip -Force -ErrorAction SilentlyContinue
Compress-Archive -Path $stage -DestinationPath $zip

$size = [math]::Round((Get-Item $zip).Length / 1MB, 2)
Write-Host "Packaged $zip ($size MB)"
