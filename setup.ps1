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

$EigenZipUrl       = "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip"
$EigenZip          = Join-Path $DownloadDir "eigen-3.4.0.zip"
$EigenExtractDir   = Join-Path $DepsDir "eigen"

$GtsamZipUrl       = "https://github.com/borglab/gtsam/archive/refs/tags/4.2.2.zip"
$GtsamZip          = Join-Path $DownloadDir "gtsam-4.2.2.zip"
$GtsamSourceDir    = Join-Path $DepsDir "gtsam-src"
$GtsamInstallDir   = Join-Path $DepsDir "gtsam-install"

# GTSAM 4.2.2 (the latest stable release) unconditionally requires Boost to build on Windows -
# there is no released version yet with the boost-optional flags seen on GTSAM's unreleased main
# branch. vcpkg is what GTSAM's own build docs recommend for getting prebuilt Boost on Windows.
# Installing the full "boost" metapackage (not just the components find_package(Boost COMPONENTS...)
# asks for) because GTSAM's .cpp/.h files directly #include several other header-only Boost
# libraries (boost/assign, boost/ptr_container, ...) that aren't pulled in by the component libs alone.
$VcpkgDir          = Join-Path $DepsDir "vcpkg"
$VcpkgExe          = Join-Path $VcpkgDir "vcpkg.exe"
$VcpkgToolchain    = Join-Path $VcpkgDir "scripts\buildsystems\vcpkg.cmake"

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

function Expand-EigenIfMissing {
    if (Test-Path (Join-Path $EigenExtractDir "Eigen\Dense")) {
        Write-Host "Eigen already extracted."
        return
    }
    Write-Host "Extracting Eigen..."
    New-Item -ItemType Directory -Force -Path $EigenExtractDir | Out-Null
    Expand-Archive -Path $EigenZip -DestinationPath $EigenExtractDir -Force
    $inner = Join-Path $EigenExtractDir "eigen-3.4.0"
    if (Test-Path $inner) {
        Get-ChildItem $inner | Move-Item -Destination $EigenExtractDir -Force
        Remove-Item $inner -Recurse -Force
    }
}

function Expand-GtsamSourceIfMissing {
    if (Test-Path (Join-Path $GtsamSourceDir "CMakeLists.txt")) {
        Write-Host "GTSAM source already extracted."
        return
    }
    Write-Host "Extracting GTSAM source..."
    New-Item -ItemType Directory -Force -Path $GtsamSourceDir | Out-Null
    Expand-Archive -Path $GtsamZip -DestinationPath $GtsamSourceDir -Force
    $inner = Join-Path $GtsamSourceDir "gtsam-4.2.2"
    if (Test-Path $inner) {
        Get-ChildItem $inner | Move-Item -Destination $GtsamSourceDir -Force
        Remove-Item $inner -Recurse -Force
    }
}

function Install-VcpkgIfMissing {
    if (Test-Path $VcpkgExe) {
        Write-Host "vcpkg already bootstrapped."
        return
    }
    Write-Host "Cloning + bootstrapping vcpkg (used only to get a prebuilt Boost for GTSAM)..."
    git clone --depth 1 https://github.com/microsoft/vcpkg.git $VcpkgDir
    & (Join-Path $VcpkgDir "bootstrap-vcpkg.bat")
}

function Install-BoostIfMissing {
    Write-Host "Installing full Boost via vcpkg (large - GTSAM's source pulls in more than just the CMake-required components)..."
    & $VcpkgExe install boost --triplet x64-windows
}

function Build-GtsamIfMissing {
    if (Test-Path (Join-Path $GtsamInstallDir "lib\cmake\GTSAM\GTSAMConfig.cmake")) {
        Write-Host "GTSAM already built and installed."
        return
    }
    Write-Host "Configuring + building GTSAM (this is large - expect a long build)..."
    $gtsamBuildDir = Join-Path $GtsamSourceDir "build"
    cmake -S $GtsamSourceDir -B $gtsamBuildDir -A x64 `
        -DCMAKE_TOOLCHAIN_FILE="$VcpkgToolchain" `
        -DVCPKG_TARGET_TRIPLET=x64-windows `
        -DCMAKE_INSTALL_PREFIX="$GtsamInstallDir" `
        -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 `
        -DGTSAM_USE_SYSTEM_EIGEN=OFF `
        -DGTSAM_BUILD_TESTS=OFF `
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF `
        -DGTSAM_BUILD_UNSTABLE=OFF   # gtsam_unstable hits duplicate-symbol LNK2005 errors as a
                                     # shared lib on MSVC (LPInitSolver's std::map<Key,Key>
                                     # instantiation clashes with gtsam.dll's). We don't need
                                     # anything from gtsam_unstable, so just skip building it.
    cmake --build $gtsamBuildDir --config Release --target install
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
Get-FileIfMissing $EigenZipUrl $EigenZip
Get-FileIfMissing $GtsamZipUrl $GtsamZip

Write-Step "Extracting dependencies"
Expand-DepthaiIfMissing
Expand-OpenCvIfMissing
Expand-EigenIfMissing
Expand-GtsamSourceIfMissing

Write-Step "Setting up vcpkg + Boost (needed by GTSAM)"
Install-VcpkgIfMissing
Install-BoostIfMissing

Write-Step "Building GTSAM"
Build-GtsamIfMissing

Write-Step "Configuring project"
$PrefixPath = "$DepthaiExtractDir;$OpenCvExtractDir\build"
# GTSAMConfig.cmake itself calls find_dependency(Boost), so the project configure
# needs the vcpkg toolchain too, not just GTSAM's own build.
cmake -S $RepoDir -B "$RepoDir\build" -A x64 -DCMAKE_PREFIX_PATH="$PrefixPath" `
    -DCMAKE_TOOLCHAIN_FILE="$VcpkgToolchain" -DVCPKG_TARGET_TRIPLET=x64-windows

Write-Step "Building all targets (Release)"
cmake --build "$RepoDir\build" --config Release

Write-Step "Done"
$exe = Join-Path $RepoDir "build\Release\VIO.exe"
Write-Host "Binary (self-contained, all DLLs copied alongside it): $exe"
Write-Host "Run it with: & `"$exe`""
