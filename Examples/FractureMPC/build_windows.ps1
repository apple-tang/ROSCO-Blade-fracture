param(
    [string]$CMake = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe',
    [string]$Fortran = 'C:\Program Files (x86)\Intel\oneAPI\compiler\latest\bin\ifx.exe',
    [string]$QpOasesSource = 'C:\Dev\qpOASES'
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$qpBuild = Join-Path $repo 'rosco/controller/build-fracture-qp'
$controllerBuild = Join-Path $repo 'rosco/controller/build-fracture-mpc'
& $CMake -S $QpOasesSource -B $qpBuild -G 'Visual Studio 17 2022' -A x64 -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded -DQPOASES_BUILD_EXAMPLES=OFF
if ($LASTEXITCODE) { throw 'qpOASES configuration failed' }
& $CMake --build $qpBuild --config Release --parallel 4
if ($LASTEXITCODE) { throw 'qpOASES build failed' }
& $CMake -S (Join-Path $repo 'rosco/controller') -B $controllerBuild -G 'Visual Studio 17 2022' -A x64 "-DCMAKE_Fortran_COMPILER=$Fortran" '-DCMAKE_Fortran_FLAGS=-libs:static -static -free -fpp -real-size:64 -double-size:64' -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded -DROSCO_FRACTURE_MPC=ON "-DQPOASES_INCLUDE_DIR=$QpOasesSource/include" "-DQPOASES_LIBRARY=$qpBuild/libs/Release/qpOASES.lib"
if ($LASTEXITCODE) { throw 'ROSCO configuration failed' }
& $CMake --build $controllerBuild --config Release --parallel 4
if ($LASTEXITCODE) { throw 'ROSCO build failed' }
