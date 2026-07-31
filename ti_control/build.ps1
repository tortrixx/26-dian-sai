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
Build vision-only ball-balance firmware for LP-MSPM0G3507.
Output: msp_control.out
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Require-Path([string]$Path, [string]$Name) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Name not found: $Path"
    }
}

$projectRoot = $PSScriptRoot
$compiler   = Join-Path $CompilerRoot "bin\tiarmclang.exe"
$startup    = Join-Path $SdkRoot "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c"
$cmsisInc   = Join-Path $SdkRoot "source\third_party\CMSIS\Core\Include"
$srcInc     = Join-Path $SdkRoot "source"
$deviceOpt  = Join-Path $projectRoot "device.opt"

Require-Path $compiler   "TI Arm Clang compiler"
Require-Path $SysConfigCli "SysConfig CLI"
Require-Path $startup    "MSPM0G3507 startup"
Require-Path (Join-Path $SdkRoot ".metadata\product.json") "MSPM0 SDK"

Push-Location $projectRoot
try {
    # ---- SysConfig generation ----
    & $SysConfigCli --compiler ticlang `
        --product (Join-Path $SdkRoot ".metadata\product.json") `
        --output . msp_control.syscfg
    if ($LASTEXITCODE -ne 0) { throw "SysConfig failed ($LASTEXITCODE)." }

    $cOpts = @(
        "-I.", "@device.opt",
        "-I$cmsisInc", "-I$srcInc",
        "-gdwarf-3", "-mcpu=cortex-m0plus", "-march=thumbv6m",
        "-mfloat-abi=soft", "-mthumb", "-Wall", "-O2"
    )

    # ---- Compile ----
    $srcFiles = @(
        "empty.c", "app.c", "buttons.c", "encoder.c", "k230_uart.c",
        "line_follow.c", "line_sensor.c", "menu.c", "motor.c",
        "oled.c", "servo.c", "static_ball.c", "system_time.c",
        "ti_msp_dl_config.c", "startup_mspm0g350x_ticlang.c"
    )
    $objs = @()
    foreach ($s in $srcFiles) {
        $o = [System.IO.Path]::ChangeExtension($s, ".obj")
        & $compiler @cOpts -c $s -o $o
        if ($LASTEXITCODE -ne 0) { throw "$s compile failed ($LASTEXITCODE)." }
        $objs += $o
    }

    # ---- Link ----
    $linkOpts = @(
        "-Wl,-u,_c_int00"
    ) + $objs + @(
        "-ldevice.cmd.genlibs",
        "-L$srcInc", "-L.",
        "device_linker.cmd",
        "-Wl,-m,msp_control.map",
        "-Wl,--rom_model", "-Wl,--warn_sections",
        "-L$(Join-Path $CompilerRoot 'lib')",
        "-llibc.a",
        "-o", "msp_control.out"
    )

    & $compiler @linkOpts
    if ($LASTEXITCODE -ne 0) { throw "Link failed ($LASTEXITCODE)." }

    Write-Host "BUILD OK: $projectRoot\msp_control.out"
} finally {
    Pop-Location
}
