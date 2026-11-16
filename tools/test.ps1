param([string]$Compiler = 'C:/mingw64/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectDir '.build'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' "-I$projectDir" `
    "$projectDir/tests/car_test.c" "$projectDir/car.c" "$projectDir/pid.c" "$projectDir/drive.c" '-lm' '-o' "$buildDir/car_test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
& "$buildDir/car_test.exe" | Tee-Object -FilePath "$buildDir/control-tests.log"
if ($LASTEXITCODE -ne 0) { throw 'Control logic tests failed' }
