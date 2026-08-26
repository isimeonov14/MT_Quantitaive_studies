from pathlib import Path
import csv
import math


# =============================================================================
# Configuration
# =============================================================================

SOURCE_RATE_HZ = 120
REPLAY_RATE_HZ = 60

QUAT_NORM_WARN_TOLERANCE = 0.01


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

RAW_DATA = ROOT_DIR / "raw_data.csv"

LOG_DIR = ROOT_DIR / "logs"
INPUT_LOG = LOG_DIR / "renode_uart.log"

GENERATED_DIR = ROOT_DIR / "generated_source"
OUTPUT_H = GENERATED_DIR / "quat_replay_data.h"
OUTPUT_C = GENERATED_DIR / "quat_replay_data.c"


# =============================================================================
# Helpers
# =============================================================================

def count_csv_samples(path: Path) -> int:
    """Count data rows in the original source CSV."""
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)

        try:
            next(reader)  # header
        except StopIteration:
            return 0

        return sum(1 for row in reader if row)


def parse_uart_quaternions(path: Path):
    """
    Parse records of the form:

        QUAT,index,timestamp,x,y,z,w

    Other UART output is ignored.
    """
    samples = []

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line_number, raw_line in enumerate(f, start=1):

            # Remove common UART artefacts.
            line = raw_line.replace("\x00", "").strip()

            # Allow ordinary Zephyr/logging output in the same UART file.
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

                x = float(fields[3])
                y = float(fields[4])
                z = float(fields[5])
                w = float(fields[6])

            except ValueError as exc:
                raise RuntimeError(
                    f"Invalid QUAT value at UART line {line_number}:\n"
                    f"{record}"
                ) from exc

            samples.append(
                {
                    "index": index,
                    "timestamp": timestamp,
                    "x": x,
                    "y": y,
                    "z": z,
                    "w": w,
                }
            )

    return samples


def validate_samples(samples, expected_count: int):
    """Validate sequence continuity and quaternion values."""

    if not samples:
        raise RuntimeError(
            f"No QUAT records found in:\n{INPUT_LOG}"
        )

    # -------------------------------------------------------------------------
    # Sequence continuity
    # -------------------------------------------------------------------------

    for expected_index, sample in enumerate(samples):

        if sample["index"] != expected_index:
            raise RuntimeError(
                "Quaternion sequence discontinuity:\n"
                f"  expected index: {expected_index}\n"
                f"  received index: {sample['index']}"
            )

    # -------------------------------------------------------------------------
    # Expected sample count
    #
    # The source is 120 Hz and firmware output is 60 Hz:
    #
    #     2 source samples -> 1 application-facing sample
    # -------------------------------------------------------------------------

    if len(samples) != expected_count:
        raise RuntimeError(
            "Unexpected quaternion sample count:\n"
            f"  expected: {expected_count}\n"
            f"  received: {len(samples)}"
        )

    # -------------------------------------------------------------------------
    # Quaternion values
    # -------------------------------------------------------------------------

    norms = []
    warning_count = 0

    for sample in samples:

        q = (
            sample["x"],
            sample["y"],
            sample["z"],
            sample["w"],
        )

        if not all(math.isfinite(v) for v in q):
            raise RuntimeError(
                f"Non-finite quaternion at index {sample['index']}"
            )

        norm = math.sqrt(sum(v * v for v in q))
        norms.append(norm)

        if abs(norm - 1.0) > QUAT_NORM_WARN_TOLERANCE:
            warning_count += 1

    return norms, warning_count


def generate_header():
    """Generate quat_replay_data.h."""

    text = f"""\
#ifndef QUAT_REPLAY_DATA_H_
#define QUAT_REPLAY_DATA_H_

#include <stddef.h>

#define QUAT_REPLAY_RATE_HZ {REPLAY_RATE_HZ}U

/*
 * SmartVNS quaternion representation.
 *
 * Component order:
 *     [x, y, z, w]
 */
struct quat_replay_sample
{{
    float x;
    float y;
    float z;
    float w;
}};

extern const struct quat_replay_sample quat_replay_data[];
extern const size_t quat_replay_data_count;

#endif /* QUAT_REPLAY_DATA_H_ */
"""

    OUTPUT_H.write_text(text, encoding="utf-8")


def generate_source(samples):
    """Generate quat_replay_data.c."""

    lines = [
        '#include "quat_replay_data.h"',
        "",
        "const struct quat_replay_sample quat_replay_data[] =",
        "{",
    ]

    for sample in samples:

        # 9 significant decimal digits are sufficient to round-trip
        # IEEE-754 binary32 values.
        lines.append(
            "    { "
            f".x = {sample['x']:.9g}f, "
            f".y = {sample['y']:.9g}f, "
            f".z = {sample['z']:.9g}f, "
            f".w = {sample['w']:.9g}f "
            "},"
        )

    lines += [
        "};",
        "",
        "const size_t quat_replay_data_count =",
        "    sizeof(quat_replay_data) / sizeof(quat_replay_data[0]);",
        "",
    ]

    OUTPUT_C.write_text(
        "\n".join(lines),
        encoding="utf-8",
    )


# =============================================================================
# Main
# =============================================================================

def main():

    print("=== SmartVNS Stage 1 UART Parser ===")
    print()

    # -------------------------------------------------------------------------
    # Check inputs
    # -------------------------------------------------------------------------

    if not RAW_DATA.exists():
        raise FileNotFoundError(
            f"Source CSV not found:\n{RAW_DATA}"
        )

    if not INPUT_LOG.exists():
        raise FileNotFoundError(
            f"Renode UART log not found:\n{INPUT_LOG}"
        )

    GENERATED_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    # -------------------------------------------------------------------------
    # Determine expected output size
    # -------------------------------------------------------------------------

    source_samples = count_csv_samples(RAW_DATA)

    if source_samples == 0:
        raise RuntimeError("raw_data.csv contains no samples.")

    expected_samples = (
        source_samples * REPLAY_RATE_HZ
        // SOURCE_RATE_HZ
    )

    print(f"Source CSV:       {RAW_DATA}")
    print(f"UART log:         {INPUT_LOG}")
    print()
    print(f"Source samples:   {source_samples}")
    print(f"Source rate:      {SOURCE_RATE_HZ} Hz")
    print(f"Quaternion rate:  {REPLAY_RATE_HZ} Hz")
    print(f"Expected output:  {expected_samples}")
    print()

    # -------------------------------------------------------------------------
    # Parse UART
    # -------------------------------------------------------------------------

    samples = parse_uart_quaternions(INPUT_LOG)

    # -------------------------------------------------------------------------
    # Validate
    # -------------------------------------------------------------------------

    norms, warning_count = validate_samples(
        samples,
        expected_samples,
    )

    print(f"Parsed samples:   {len(samples)}")
    print()
    print("Quaternion norm:")
    print(f"  minimum:        {min(norms):.8f}")
    print(f"  mean:           {sum(norms) / len(norms):.8f}")
    print(f"  maximum:        {max(norms):.8f}")

    if warning_count:
        print()
        print(
            f"WARNING: {warning_count} quaternion samples differ "
            f"from unit norm by more than "
            f"{QUAT_NORM_WARN_TOLERANCE:.3f}."
        )

    # -------------------------------------------------------------------------
    # Generate BabbleSim source
    # -------------------------------------------------------------------------

    generate_header()
    generate_source(samples)

    print()
    print("Generated successfully:")
    print(f"  {OUTPUT_H}")
    print(f"  {OUTPUT_C}")
    print()
    print(
        f"{len(samples)} quaternion samples at "
        f"{REPLAY_RATE_HZ} Hz ready for BabbleSim."
    )


if __name__ == "__main__":
    main()