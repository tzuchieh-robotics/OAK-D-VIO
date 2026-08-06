# One-shot setup: downloads depthai-core (v3.8.0 win64 prebuilt) and OpenCV 4.13.0,
# then configures + builds HelloWorld.cpp against them.
# Run from a normal PowerShell window (no admin rights needed).
#
# OpenCV version note: depthai-core.dll (the prebuilt Luxonis release) was built
# against OpenCV 4.13.0 specifically - it imports opencv_world4130.dll by exact
# name. Using any other OpenCV version here will build fine but fail at runtime
# with STATUS_DLL_NOT_FOUND (0xC0000135).

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

$RepoDir     = $PSScriptRoot
$DepsDir     = Join-Path $RepoDir "deps"
$DownloadDir = Join-Path $DepsDir "_downloads"

$DepthaiZipUrl     = "https://github.com/luxonis/depthai-core/releases/download/v3.8.0/depthai-core-v3.8.0-win64.zip"
$DepthaiZip        = Join-Path $DownloadDir "depthai-core-v3.8.0-win64.zip"
$DepthaiExtractDir = Join-Path $DepsDir "depthai-core"

$OpenCvExeUrl      = "https://github.com/opencv/opencv/releases/download/4.13.0/opencv-4.13.0-windows.exe"
$OpenCvExe         = Join-Path $DownloadDir "opencv-4.13.0-windows.exe"
$OpenCvExtractDir  = Join-Path $DepsDir "opencv"

function Write-Step($msg) {
    Write-Host ""
    Write-Host "==> $msg" -ForegroundColor Cyan
}

function Test-Prereqs {
    Write-Step "Checking prerequisites (CMake, Visual Studio C++ toolset)"

    $cmake = Get-Command cmake.exe -ErrorAction SilentlyContinue
    if (-not $cmake) {
        Write-Host "CMake not found on PATH." -ForegroundColor Red
        Write-Host "Install it from https://cmake.org/download/ (check 'Add CMake to PATH'), then re-run this script." -ForegroundColor Yellow
        exit 1
    }
    Write-Host "Found cmake: $($cmake.Source)"

    $vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
    $hasVc = $false
    if (Test-Path $vswhere) {
        $vsInstall = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($vsInstall) { $hasVc = $true }
    }
    if (-not $hasVc) {
        Write-Host "Visual Studio with the 'Desktop development with C++' workload not found." -ForegroundColor Red
        Write-Host "Install from https://visualstudio.microsoft.com/downloads/ , then re-run this script." -ForegroundColor Yellow
        exit 1
    }
    Write-Host "Found Visual Studio C++ toolset."
}

function Get-FileIfMissing($Url, $Dest) {
    if (Test-Path $Dest) {
        Write-Host "Already downloaded: $Dest"
        return
    }
    Write-Host "Downloading $Url"
    Invoke-WebRequest -Uri $Url -OutFile $Dest
}

function Expand-DepthaiIfMissing {
    if (Test-Path (Join-Path $DepthaiExtractDir "lib\cmake\depthai\depthaiConfig.cmake")) {
        Write-Host "depthai-core already extracted."
        return
    }
    Write-Host "Extracting depthai-core..."
    New-Item -ItemType Directory -Force -Path $DepthaiExtractDir | Out-Null
    Expand-Archive -Path $DepthaiZip -DestinationPath $DepthaiExtractDir -Force
    $inner = Join-Path $DepthaiExtractDir "depthai-core-v3.8.0-win64"
    if (Test-Path $inner) {
        Get-ChildItem $inner | Move-Item -Destination $DepthaiExtractDir -Force
        Remove-Item $inner -Recurse -Force
    }
}

function Expand-OpenCvIfMissing {
    if (Test-Path (Join-Path $OpenCvExtractDir "build\OpenCVConfig.cmake")) {
        Write-Host "OpenCV already extracted."
        return
    }
    Write-Host "Extracting OpenCV (self-extracting archive)..."
    New-Item -ItemType Directory -Force -Path $OpenCvExtractDir | Out-Null
    $argStr = '-o"' + $OpenCvExtractDir + '" -y'
    Start-Process -FilePath $OpenCvExe -ArgumentList $argStr -Wait
    # Archive contains a top-level "opencv" folder - flatten it so paths match CMakeLists.txt.
    $inner = Join-Path $OpenCvExtractDir "opencv"
    if (Test-Path $inner) {
        Get-ChildItem $inner | Move-Item -Destination $OpenCvExtractDir -Force
        Remove-Item $inner -Recurse -Force
    }
}

# --- main ---

Test-Prereqs

New-Item -ItemType Directory -Force -Path $DownloadDir | Out-Null

Write-Step "Downloading dependencies"
Get-FileIfMissing $DepthaiZipUrl $DepthaiZip
Get-FileIfMissing $OpenCvExeUrl $OpenCvExe

Write-Step "Extracting dependencies"
Expand-DepthaiIfMissing
Expand-OpenCvIfMissing

Write-Step "Configuring project"
$PrefixPath = "$DepthaiExtractDir;$OpenCvExtractDir\build"
cmake -S $RepoDir -B "$RepoDir\build" -A x64 -DCMAKE_PREFIX_PATH="$PrefixPath"

Write-Step "Building HelloWorld (Release)"
cmake --build "$RepoDir\build" --config Release

Write-Step "Done"
$exe = Join-Path $RepoDir "build\Release\HelloWorld.exe"
Write-Host "Binary (self-contained, all DLLs copied alongside it): $exe"
Write-Host "Run it with: & `"$exe`""
