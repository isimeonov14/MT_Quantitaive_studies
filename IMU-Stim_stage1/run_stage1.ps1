param(
    [string]$RenodeExe = "D:\Renode\renode.exe"
)

$ErrorActionPreference = "Stop"

# ============================================================================
# Paths
# ============================================================================

# Folder containing this script
$Root = $PSScriptRoot

$RenodeDir   = Join-Path $Root "Renode"
$LogsDir     = Join-Path $Root "logs"
$GeneratedDir = Join-Path $Root "generated_source"
$ScriptsDir  = Join-Path $Root "scripts"

$Resc   = Join-Path $RenodeDir "log_uart.resc"
$Uart   = Join-Path $LogsDir "renode_uart.log"
$Parser = Join-Path $ScriptsDir "uart_to_quat_c.py"

Write-Host "=== SmartVNS Renode Stage 1 ==="
Write-Host "Root: $Root"
Write-Host ""

# ============================================================================
# Create output directories
# ============================================================================

New-Item -ItemType Directory -Force -Path $LogsDir | Out-Null
New-Item -ItemType Directory -Force -Path $GeneratedDir | Out-Null

# ============================================================================
# Check required files
# ============================================================================

$RequiredFiles = @(
    $RenodeExe,
    $Resc,
    $Parser,
    (Join-Path $Root "raw_data.csv"),
    (Join-Path $RenodeDir "zephyr.elf"),
    (Join-Path $RenodeDir "tracker_spi_imu.repl"),
    (Join-Path $RenodeDir "LSM6DSV16BX_SmartVNS_Replay.cs")
)

foreach ($File in $RequiredFiles) {
    if (-not (Test-Path $File)) {
        throw "Required file not found: $File"
    }
}

# ============================================================================
# Clean previous UART output
# ============================================================================

if (Test-Path $Uart) {
    Write-Host "Removing previous UART capture..."
    Remove-Item $Uart -Force
}

# ============================================================================
# Run Renode
# ============================================================================

Write-Host "Running Renode..."

& $RenodeExe `
    --disable-gui `
    --console `
    $Resc

if ($LASTEXITCODE -ne 0) {
    throw "Renode failed with exit code $LASTEXITCODE"
}

# ============================================================================
# Check UART capture
# ============================================================================

if (-not (Test-Path $Uart)) {
    throw "UART log was not created: $Uart"
}

$UartSize = (Get-Item $Uart).Length

if ($UartSize -eq 0) {
    throw "UART log is empty."
}

Write-Host "UART capture complete: $UartSize bytes"

# ============================================================================
# Convert UART trace -> C replay source
# ============================================================================

Write-Host "Generating BabbleSim quaternion source..."

python $Parser

if ($LASTEXITCODE -ne 0) {
    throw "Quaternion parser failed with exit code $LASTEXITCODE"
}

# ============================================================================
# Check generated files
# ============================================================================

$Header = Join-Path $GeneratedDir "quat_replay_data.h"
$Source = Join-Path $GeneratedDir "quat_replay_data.c"

if (-not (Test-Path $Header)) {
    throw "Generated header not found: $Header"
}

if (-not (Test-Path $Source)) {
    throw "Generated C source not found: $Source"
}

Write-Host ""
Write-Host "========================================"
Write-Host " Stage 1 completed successfully"
Write-Host "========================================"
Write-Host ""
Write-Host "UART log:"
Write-Host "  $Uart"
Write-Host ""
Write-Host "BabbleSim source:"
Write-Host "  $Header"
Write-Host "  $Source"