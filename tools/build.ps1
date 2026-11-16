param(
    [string]$Sdk = 'C:/TI/mspm0_sdk_2_11_00_07',
    [string]$SysConfig = 'D:/ti/ccs2050/sysconfig_1.26.2',
    [string]$Compiler = 'D:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS'
)
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectDir '.build'
$generatedDir = Join-Path $buildDir 'syscfg'
$objectDir = Join-Path $buildDir 'obj'
New-Item -ItemType Directory -Path $generatedDir,$objectDir -Force | Out-Null
# SysConfig 的产品缓存也放在项目里，避免依赖用户目录写权限。
$originalLocalAppData = $env:LOCALAPPDATA
try {
    $env:LOCALAPPDATA = Join-Path $buildDir 'localappdata'
    New-Item -ItemType Directory -Path "$env:LOCALAPPDATA/TI_pinmux" -Force | Out-Null
    & "$SysConfig/sysconfig_cli.bat" -s "$Sdk/.metadata/product.json" --script "$projectDir/empty.syscfg" -o $generatedDir --compiler ticlang
    if ($LASTEXITCODE -ne 0) { throw 'SysConfig failed' }
} finally { $env:LOCALAPPDATA = $originalLocalAppData }

$cc = "$Compiler/bin/tiarmclang.exe"
$flags = @('-march=thumbv6m','-mcpu=cortex-m0plus','-mfloat-abi=soft','-mlittle-endian','-mthumb',
    '-O1','-g','-Wall','-Wextra','-Werror',"@$generatedDir/device.opt",
    "-I$projectDir","-I$generatedDir","-I$Sdk/source/third_party/CMSIS/Core/Include","-I$Sdk/source")
$sourcePaths = @(Get-ChildItem -LiteralPath $projectDir -Filter '*.c' | ForEach-Object { $_.FullName })
$sourcePaths += "$generatedDir/ti_msp_dl_config.c"
$sourcePaths += "$Sdk/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c"
$objects = @()
foreach ($sourcePath in $sourcePaths) {
    $objectPath = Join-Path $objectDir (([IO.Path]::GetFileNameWithoutExtension($sourcePath)) + '.o')
    & $cc @flags -c $sourcePath -o $objectPath
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $sourcePath" }
    $objects += $objectPath
}
Push-Location $generatedDir
try {
    $firmware = Join-Path $buildDir 'h2024_car.out'
    & $cc @flags @objects "-Wl,-i$Sdk/source" "-Wl,-i$Compiler/lib" '-Wl,-ldevice_linker.cmd' `
        '-Wl,-ldevice.cmd.genlibs' '-Wl,-llibc.a' '-Wl,--rom_model' '-Wl,--stack_size=2048' `
        "-Wl,-m$buildDir/h2024_car.map" -o $firmware
    if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
    & "$Compiler/bin/tiarmobjcopy.exe" -O binary $firmware "$buildDir/h2024_car.bin"
    if ($LASTEXITCODE -ne 0) { throw 'Binary export failed' }
    Write-Output "Built: $firmware"
} finally { Pop-Location }
