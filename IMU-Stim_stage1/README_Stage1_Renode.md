# Stage 1 – Renode IMU Replay

This stage replays recorded IMU data through the SmartVNS tracker firmware in Renode and generates quaternion data for the next BabbleSim stage.

## What it does

- Loads `raw_data.csv` as a 120 Hz recorded IMU source.
- Replays the IMU through the Renode `LSM6DSV16BX` model.
- Runs the production tracker IMU processing at 60 Hz.
- Runs the production VQF orientation filter.
- Captures quaternion output from UART in the format:

  `QUAT,index,timestamp,x,y,z,w`

- Parses the UART log and generates:
  - `generated_source/quat_replay_data.h`
  - `generated_source/quat_replay_data.c`

The generated quaternion data uses the SmartVNS order `[x, y, z, w]` and is intended as input to the BabbleSim tracker application.

## Folder structure

```text
.
│   raw_data.csv
│   run_stage1.ps1
│
├── generated_source/
├── logs/
├── Renode/
│   ├── log_uart.resc
│   ├── LSM6DSV16BX_SmartVNS_Replay.cs
│   ├── NRF5340_SPI_CS.cs
│   ├── tracker_spi_imu.repl
│   └── zephyr.elf
└── scripts/
    └── uart_to_quat_c.py
```

## Run

From PowerShell:

```powershell
.\run_stage1.ps1
```

The script:

1. Removes the previous UART log.
2. Runs Renode headlessly.
3. Saves UART output to `logs/renode_uart.log`.
4. Runs `scripts/uart_to_quat_c.py`.
5. Validates the quaternion sequence and sample count.
6. Writes the generated C source files to `generated_source/`.

## Output

After a successful run:

```text
logs/renode_uart.log

generated_source/quat_replay_data.h
generated_source/quat_replay_data.c
```

The generated C files are passed to the BabbleSim tracker application for BLE replay.
