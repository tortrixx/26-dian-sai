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
Build the standalone LP-MSPM0G3507 MPU-6050 electrical/I2C test.

The script intentionally does not flash the board.  Use the CCS debugger or
DSLite with mspm0g3507.ccxml only after reviewing the wiring in README.md.
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

    & $compiler @compileOptions -c msp_mpu6050_test.c -o msp_mpu6050_test.obj
    if ($LASTEXITCODE -ne 0) { throw "Application compilation failed ($LASTEXITCODE)." }

    & $compiler @compileOptions -c ti_msp_dl_config.c -o ti_msp_dl_config.obj
    if ($LASTEXITCODE -ne 0) { throw "SysConfig compilation failed ($LASTEXITCODE)." }

    & $compiler @compileOptions -c $startup -o startup_mspm0g350x_ticlang.obj
    if ($LASTEXITCODE -ne 0) { throw "Startup compilation failed ($LASTEXITCODE)." }

    $linkOptions = @(
        "-Wl,-u,_c_int00",
        "msp_mpu6050_test.obj",
        "ti_msp_dl_config.obj",
        "startup_mspm0g350x_ticlang.obj",
        "-ldevice.cmd.genlibs",
        "-L$sourceInclude",
        "-L.",
        "device_linker.cmd",
        "-Wl,-m,msp_mpu6050_test.map",
        "-Wl,--rom_model",
        "-Wl,--warn_sections",
        "-L$(Join-Path $CompilerRoot 'lib')",
        "-llibc.a",
        "-o",
        "msp_mpu6050_test.out"
    )

    & $compiler @linkOptions
    if ($LASTEXITCODE -ne 0) { throw "Linking failed ($LASTEXITCODE)." }

    Write-Host "Build succeeded: $projectRoot\msp_mpu6050_test.out"
}
finally {
    Pop-Location
}
