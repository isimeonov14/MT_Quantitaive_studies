param(
    [string]$RenodeExe = "D:\Renode\renode.exe",
    [int]$Runs = 10
)

$ErrorActionPreference = "Stop"

function Get-Stats {
    param([double[]]$Values)

    if (-not $Values -or $Values.Count -eq 0) {
        return [pscustomobject]@{
            Mean = 0.0
            StdDev = 0.0
        }
    }

    $mean = ($Values | Measure-Object -Average).Average
    $variance = 0.0

    foreach ($v in $Values) {
        $variance += (($v - $mean) * ($v - $mean))
    }

    if ($Values.Count -gt 1) {
        $std = [Math]::Sqrt($variance / ($Values.Count - 1))
    }
    else {
        $std = 0.0
    }

    return [pscustomobject]@{
        Mean = [double]$mean
        StdDev = [double]$std
    }
}

function Get-RenodeExecutionSeconds {
    param([string[]]$Lines)

    $start = $null
    $end = $null

    foreach ($line in $Lines) {
        if ($line -match '^(?<ts>\d{2}:\d{2}:\d{2}\.\d+)\s+\[INFO\].*Machine started') {
            $start = [TimeSpan]::ParseExact($Matches['ts'], 'hh\:mm\:ss\.ffff', [System.Globalization.CultureInfo]::InvariantCulture)
        }

        if ($line -match '^(?<ts>\d{2}:\d{2}:\d{2}\.\d+)\s+\[INFO\].*Machine paused') {
            $end = [TimeSpan]::ParseExact($Matches['ts'], 'hh\:mm\:ss\.ffff', [System.Globalization.CultureInfo]::InvariantCulture)
        }
    }

    if ($null -eq $start -or $null -eq $end) {
        throw "Could not find both 'Machine started' and 'Machine paused' timestamps in Renode output."
    }

    if ($end -lt $start) {
        throw "Renode end time is before start time."
    }

    return [double]($end - $start).TotalSeconds
}

function Invoke-Stage1Once {
    param(
        [string]$RenodeExe,
        [string]$Resc,
        [string]$Parser,
        [string]$LogsDir,
        [string]$GeneratedDir,
        [string]$Uart,
        [string]$Header,
        [string]$Source
    )

    New-Item -ItemType Directory -Force -Path $LogsDir | Out-Null
    New-Item -ItemType Directory -Force -Path $GeneratedDir | Out-Null

    if (Test-Path $Uart) {
        Remove-Item $Uart -Force
    }

    $renodeOutput = @()
    $clockTime = Measure-Command {
        & $RenodeExe `
            --disable-gui `
            --console `
            $Resc `
            2>&1 |
            Tee-Object -Variable renodeOutput | Out-Null
    }

    if ($LASTEXITCODE -ne 0) {
        throw "Renode failed with exit code $LASTEXITCODE"
    }

    if (-not (Test-Path $Uart)) {
        throw "UART log was not created: $Uart"
    }

    if ((Get-Item $Uart).Length -eq 0) {
        throw "UART log is empty: $Uart"
    }

    python $Parser
    if ($LASTEXITCODE -ne 0) {
        throw "Quaternion parser failed with exit code $LASTEXITCODE"
    }

    if (-not (Test-Path $Header)) {
        throw "Generated header not found: $Header"
    }

    if (-not (Test-Path $Source)) {
        throw "Generated C source not found: $Source"
    }

    $renodeExecutionSeconds = Get-RenodeExecutionSeconds -Lines $renodeOutput

    return [pscustomobject]@{
        ClockSeconds = [double]$clockTime.TotalSeconds
        RenodeWallClockSeconds = [double]$renodeExecutionSeconds
        SimulatedSeconds = 10.0
    }
}

$Root = $PSScriptRoot
$RenodeDir = Join-Path $Root "Renode"
$LogsDir = Join-Path $Root "logs"
$GeneratedDir = Join-Path $Root "generated_source"
$ScriptsDir = Join-Path $Root "scripts"
$Resc = Join-Path $RenodeDir "log_uart.resc"
$Parser = Join-Path $ScriptsDir "uart_to_quat_c.py"
$Uart = Join-Path $LogsDir "renode_uart.log"
$Header = Join-Path $GeneratedDir "quat_replay_data.h"
$Source = Join-Path $GeneratedDir "quat_replay_data.c"
$BenchmarkDir = Join-Path $Root "benchmarks"

$RequiredFiles = @(
    $RenodeExe,
    $Resc,
    $Parser,
    (Join-Path $Root "data/no_stim_stim.csv"),
    (Join-Path $RenodeDir "zephyr.elf"),
    (Join-Path $RenodeDir "tracker_spi_imu.repl"),
    (Join-Path $RenodeDir "LSM6DSV16BX_SmartVNS_Replay.cs")
)

foreach ($File in $RequiredFiles) {
    if (-not (Test-Path $File)) {
        throw "Required file not found: $File"
    }
}

New-Item -ItemType Directory -Force -Path $BenchmarkDir | Out-Null

$results = @()

Write-Host "=== Stage 1 benchmark: $Runs runs ==="
for ($i = 1; $i -le $Runs; $i++) {
    Write-Host "Running benchmark $i/$Runs..."

    $baseResult = Invoke-Stage1Once -RenodeExe $RenodeExe -Resc $Resc -Parser $Parser -LogsDir $LogsDir -GeneratedDir $GeneratedDir -Uart $Uart -Header $Header -Source $Source
    $runResult = [pscustomobject]@{
        Run = $i
        ClockSeconds = [double]$baseResult.ClockSeconds
        RenodeWallClockSeconds = [double]$baseResult.RenodeWallClockSeconds
        SimulatedSeconds = [double]$baseResult.SimulatedSeconds
    }
    $results += $runResult

    Write-Host ("  Clock time: {0:F3}s" -f $runResult.ClockSeconds)
    Write-Host ("  Renode wall-clock execution: {0:F3}s" -f $runResult.RenodeWallClockSeconds)
    Write-Host ("  Simulated Renode time requested: {0:F3}s" -f $runResult.SimulatedSeconds)
    Write-Host ""
}

$clockValues = @($results | ForEach-Object { [double]$_.ClockSeconds })
$renodeValues = @($results | ForEach-Object { [double]$_.RenodeWallClockSeconds })

$clockStats = Get-Stats -Values $clockValues
$renodeStats = Get-Stats -Values $renodeValues

$summary = [pscustomobject]@{
    Runs = $Runs
    ClockMeanSeconds = [double]$clockStats.Mean
    ClockStdDevSeconds = [double]$clockStats.StdDev
    RenodeMeanSeconds = [double]$renodeStats.Mean
    RenodeStdDevSeconds = [double]$renodeStats.StdDev
    SimulatedSeconds = 10.0
}

$rawCsv = Join-Path $BenchmarkDir "stage1_benchmark_runs.csv"
$summaryCsv = Join-Path $BenchmarkDir "stage1_benchmark_summary.csv"

$results | Select-Object Run, ClockSeconds, RenodeWallClockSeconds, SimulatedSeconds | Export-Csv -Path $rawCsv -NoTypeInformation
$summary | Select-Object Runs, ClockMeanSeconds, ClockStdDevSeconds, RenodeMeanSeconds, RenodeStdDevSeconds, SimulatedSeconds | Export-Csv -Path $summaryCsv -NoTypeInformation

Write-Host "=== Benchmark summary ==="
Write-Host ("Runs: {0}" -f $Runs)
Write-Host ("Clock time mean: {0:F3}s" -f $clockStats.Mean)
Write-Host ("Clock time std (sample): {0:F3}s" -f $clockStats.StdDev)
Write-Host ("Renode wall-clock mean: {0:F3}s" -f $renodeStats.Mean)
Write-Host ("Renode wall-clock std (sample): {0:F3}s" -f $renodeStats.StdDev)
Write-Host ("Requested Renode simulated time: {0:F3}s" -f 10.0)
Write-Host ""
Write-Host "Raw CSV: $rawCsv"
Write-Host "Summary CSV: $summaryCsv"
Write-Host ""

$results | Select-Object Run, ClockSeconds, RenodeWallClockSeconds, SimulatedSeconds | Format-Table -AutoSize
