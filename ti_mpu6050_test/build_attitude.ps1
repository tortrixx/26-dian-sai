[CmdletBinding()]
param(
    [string]$SdkRoot = $(if ($env:COM_TI_MSPM0_SDK_INSTALL_DIR) {
        $env:COM_TI_MSPM0_SDK_INSTALL_DIR
    } else {
        "C:\ti\mspm0_sdk_2_10_00_04"
    }),
    [string]$CompilerRoot = $(if ($env:TICLANG_ARMCOMPILER) {
        $env:TICLANG_ARMCOMPILER
    } else {
        "C:\ti\ti_cgt_arm_llvm_4.0.2.LTS"
    }),
    [string]$SysConfigCli = "C:\ti\sysconfig_1.26.2\sysconfig_cli.bat"
)

<#
Build the MPU-6050 pendulum attitude demo.
Same SysConfig as the original test (I2C1 + UART0), plus the new mpu6050.c module.
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Require-Path([string]$Path, [string]$Name) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Name not found: $Path"
    }
}

$projectRoot = $PSScriptRoot
$compiler = Join-Path $CompilerRoot "bin\tiarmclang.exe"
$startup = Join-Path $SdkRoot "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c"
$cmsisInclude = Join-Path $SdkRoot "source\third_party\CMSIS\Core\Include"
$sourceInclude = Join-Path $SdkRoot "source"

Require-Path $compiler "TI Arm Clang compiler"
Require-Path $SysConfigCli "SysConfig CLI"
Require-Path $startup "MSPM0G3507 startup source"
Require-Path (Join-Path $SdkRoot ".metadata\product.json") "MSPM0 SDK product metadata"

Push-Location $projectRoot
try {
    & $SysConfigCli --compiler ticlang --product (Join-Path $SdkRoot ".metadata\product.json") --output . msp_mpu6050_test.syscfg
    if ($LASTEXITCODE -ne 0) { throw "SysConfig generation failed ($LASTEXITCODE)." }

    $compileOptions = @(
        "-I.",
        "@device.opt",
        "-I$cmsisInclude",
        "-I$sourceInclude",
        "-gdwarf-3",
        "-mcpu=cortex-m0plus",
        "-march=thumbv6m",
        "-mfloat-abi=soft",
        "-mthumb",
        "-Wall",
        "-O2"
    )

    $sources = @(
        "msp_mpu6050_attitude.c",
        "mpu6050.c",
        "ti_msp_dl_config.c"
    )

    $objFiles = @()
    foreach ($src in $sources) {
        $obj = [System.IO.Path]::ChangeExtension($src, ".obj")
        & $compiler @compileOptions -c $src -o $obj
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $src ($LASTEXITCODE)." }
        $objFiles += $obj
    }

    & $compiler @compileOptions -c $startup -o startup_mspm0g350x_ticlang.obj
    if ($LASTEXITCODE -ne 0) { throw "Startup compilation failed ($LASTEXITCODE)." }
    $objFiles += "startup_mspm0g350x_ticlang.obj"

    $linkOptions = @(
        "-Wl,-u,_c_int00"
    ) + $objFiles + @(
        "-ldevice.cmd.genlibs",
        "-L$sourceInclude",
        "-L.",
        "device_linker.cmd",
        "-Wl,-m,msp_mpu6050_attitude.map",
        "-Wl,--rom_model",
        "-Wl,--warn_sections",
        "-L$(Join-Path $CompilerRoot 'lib')",
        "-llibc.a",
        "-o",
        "msp_mpu6050_attitude.out"
    )

    & $compiler @linkOptions
    if ($LASTEXITCODE -ne 0) { throw "Linking failed ($LASTEXITCODE)." }

    Write-Host "Build succeeded: $projectRoot\msp_mpu6050_attitude.out"
}
finally {
    Pop-Location
}
