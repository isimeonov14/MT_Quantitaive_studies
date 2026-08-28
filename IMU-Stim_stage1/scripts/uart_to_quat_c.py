from pathlib import Path
import csv


# =============================================================================
# Configuration
# =============================================================================

SOURCE_RATE_HZ = 120
REPLAY_RATE_HZ = 60


# =============================================================================
# Project paths
#
# Stage1/
# │   raw_data.csv
# │
# ├── generated_source/
# ├── logs/
# │       renode_uart.log
# └── scripts/
#         uart_to_quat_c.py
# =============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent

# RAW_DATA = ROOT_DIR / "data" / "raw_data.csv"
RAW_DATA = ROOT_DIR / "data" / "no_stim_stim.csv"

INPUT_LOG = ROOT_DIR / "logs" / "renode_uart.log"

GENERATED_DIR = ROOT_DIR / "generated_source"
OUTPUT_H = GENERATED_DIR / "quat_replay_data.h"
OUTPUT_C = GENERATED_DIR / "quat_replay_data.c"


# =============================================================================
# Helpers
# =============================================================================

def count_csv_samples(path: Path) -> int:
    """Count data rows in the original 120-Hz source CSV."""
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)

        try:
            next(reader)  # header
        except StopIteration:
            return 0

        return sum(1 for row in reader if row)


def parse_uart_quaternions(path: Path):
    """
    Parse Stage-1 records:

        QUAT,index,timestamp,qx,qy,qz,qw

    qx..qw are signed int16 Q15 values from the production quat_data_t.
    All unrelated Zephyr/Renode UART output is ignored.
    """
    samples = []

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line_number, raw_line in enumerate(f, start=1):
            line = raw_line.replace("\x00", "").strip()

            marker = line.find("QUAT,")
            if marker < 0:
                continue

            record = line[marker:]
            fields = record.split(",")

            if len(fields) != 7:
                raise RuntimeError(
                    f"Malformed QUAT record at UART line {line_number}:\n"
                    f"{record}"
                )

            try:
                index = int(fields[1])
                timestamp = int(fields[2])
                qx = int(fields[3])
                qy = int(fields[4])
                qz = int(fields[5])
                qw = int(fields[6])
            except ValueError as exc:
                raise RuntimeError(
                    f"Invalid QUAT value at UART line {line_number}:\n"
                    f"{record}"
                ) from exc

            samples.append(
                {
                    "index": index,
                    "timestamp": timestamp,
                    "qx": qx,
                    "qy": qy,
                    "qz": qz,
                    "qw": qw,
                }
            )

    return samples


def validate_samples(samples, expected_count: int):
    """Validate sequence continuity, sample count, timestamps and Q15 bounds."""
    if not samples:
        raise RuntimeError(
            f"No QUAT records found in:\n{INPUT_LOG}"
        )

    for expected_index, sample in enumerate(samples):
        if sample["index"] != expected_index:
            raise RuntimeError(
                "Quaternion sequence discontinuity:\n"
                f"  expected index: {expected_index}\n"
                f"  received index: {sample['index']}"
            )

        if not (0 <= sample["timestamp"] <= 0xFFFFFFFF):
            raise RuntimeError(
                f"Timestamp outside uint32_t range at sample {sample['index']}: "
                f"{sample['timestamp']}"
            )

        for name in ("qx", "qy", "qz", "qw"):
            value = sample[name]
            if value < -32768 or value > 32767:
                raise RuntimeError(
                    f"{name} outside int16_t/Q15 range at sample "
                    f"{sample['index']}: {value}"
                )
    
    if len(samples) != expected_count:
        missing = expected_count - len(samples)
        difference_pct = abs(missing) / expected_count * 100.0
    
        print(
            "WARNING: Quaternion sample count differs from expected:"
        )
        print(f"  expected: {expected_count}")
        print(f"  received: {len(samples)}")
        print(f"  difference: {missing} samples ({difference_pct:.4f}%)")
        print()


def generate_header():
    """
    Generate a header that exposes the replay trace directly as the production
    SmartVNS quat_data_t type.
    """
    text = f"""\
#ifndef QUAT_REPLAY_DATA_H_
#define QUAT_REPLAY_DATA_H_

#include <stddef.h>

#include "sensors/sensors.h"

#define QUAT_REPLAY_RATE_HZ {REPLAY_RATE_HZ}U

/*
 * Stage-1 output represented with the exact SmartVNS production type.
 *
 * quat_data_t:
 *     timestamp : uint32_t
 *     x, y, z, w: int16_t Q15 quaternion components
 */
extern const quat_data_t quat_replay_data[];
extern const size_t quat_replay_data_count;

#endif /* QUAT_REPLAY_DATA_H_ */
"""
    OUTPUT_H.write_text(text, encoding="utf-8")


def generate_source(samples):
    """Generate the production-format quaternion replay array."""
    lines = [
        '#include "quat_replay_data.h"',
        "",
        "const quat_data_t quat_replay_data[] =",
        "{",
    ]

    for sample in samples:
        lines.append(
            "    { "
            f".timestamp = {sample['timestamp']}U, "
            f".x = {sample['qx']}, "
            f".y = {sample['qy']}, "
            f".z = {sample['qz']}, "
            f".w = {sample['qw']} "
            "},"
        )

    lines += [
        "};",
        "",
        "const size_t quat_replay_data_count =",
        "    sizeof(quat_replay_data) / sizeof(quat_replay_data[0]);",
        "",
    ]

    OUTPUT_C.write_text("\n".join(lines), encoding="utf-8")


# =============================================================================
# Main
# =============================================================================

def main():
    print("=== SmartVNS Stage 1 UART Parser ===")
    print()

    if not RAW_DATA.exists():
        raise FileNotFoundError(
            f"Source CSV not found:\n{RAW_DATA}"
        )

    if not INPUT_LOG.exists():
        raise FileNotFoundError(
            f"Renode UART log not found:\n{INPUT_LOG}"
        )

    GENERATED_DIR.mkdir(parents=True, exist_ok=True)

    source_samples = count_csv_samples(RAW_DATA)

    if source_samples == 0:
        raise RuntimeError("raw_data.csv contains no samples.")

    expected_samples = (
        source_samples * REPLAY_RATE_HZ
        // SOURCE_RATE_HZ
    )

    print(f"Source samples:   {source_samples}")
    print(f"Source rate:      {SOURCE_RATE_HZ} Hz")
    print(f"Quaternion rate:  {REPLAY_RATE_HZ} Hz")
    print(f"Expected output:  {expected_samples}")
    print()

    samples = parse_uart_quaternions(INPUT_LOG)
    validate_samples(samples, expected_samples)

    print(f"Parsed samples:   {len(samples)}")

    generate_header()
    generate_source(samples)

    print()
    print("Generated successfully:")
    print(f"  {OUTPUT_H}")
    print(f"  {OUTPUT_C}")
    print()
    print(
        f"{len(samples)} production-format quat_data_t samples at "
        f"{REPLAY_RATE_HZ} Hz ready for BabbleSim."
    )


if __name__ == "__main__":
    main()
