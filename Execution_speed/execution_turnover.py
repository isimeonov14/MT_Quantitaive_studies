import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# Configuration
# ============================================================

# Sweep test count.
# Multiples of 10 preserve the exact 60/20/20 distribution.
MIN_TESTS = 10
MAX_TESTS = 200
TEST_STEP = 10

# ------------------------------------------------------------
# Mean execution time PER TEST [seconds]
# Replace these values with your measurements.
# ------------------------------------------------------------

# Native_sim
NATIVE_UNIT_TIME = 0.028498
NATIVE_INTEGRATION_TIME = 3.33729
NATIVE_SYSTEM_TIME = 3.329800

# Real hardware
HW_UNIT_TIME = 5.036294
HW_INTEGRATION_TIME = 8.7673685
HW_SYSTEM_TIME = 15.753

# ------------------------------------------------------------
# Build / deployment overhead [seconds]
# ------------------------------------------------------------

# Fixed build time for the complete test run.
# If the build time is identical for both platforms, use the
# same value here.
BUILD_TIME_NATIVE = 48.410
BUILD_TIME_HW = 48.410

# Hardware flashing time.
HW_UNIT_FLASH_TIME = 2.868999
HW_INTEGRATION_FLASH_TIME = 11.494
HW_SYSTEM_FLASH_TIME = 19.124

# True:
#   every hardware test requires one flash operation
#
# False:
#   flashing is treated as one fixed operation for the entire run
FLASH_PER_TEST = True


# ============================================================
# Fixed test distribution
# ============================================================

UNIT_RATIO = 0.40
INTEGRATION_RATIO = 0.30
SYSTEM_RATIO = 0.30


# ============================================================
# Calculation
# ============================================================

test_counts = np.arange(
    MIN_TESTS,
    MAX_TESTS + TEST_STEP,
    TEST_STEP
)

native_total_times = []
hw_total_times = []

native_throughput = []
hw_throughput = []

speedup = []


for n_tests in test_counts:

    # Fixed 60/20/20 distribution
    n_unit = int(n_tests * UNIT_RATIO)
    n_integration = int(n_tests * INTEGRATION_RATIO)
    n_system = n_tests - n_unit - n_integration

    # --------------------------------------------------------
    # Native_sim
    # --------------------------------------------------------

    native_execution_time = (
        n_unit * NATIVE_UNIT_TIME
        + n_integration * NATIVE_INTEGRATION_TIME
        + n_system * NATIVE_SYSTEM_TIME
    )

    native_total = (
        BUILD_TIME_NATIVE
        + native_execution_time
    )

    # --------------------------------------------------------
    # Hardware
    # --------------------------------------------------------

    hw_execution_time = (
        n_unit * HW_UNIT_TIME
        + n_integration * HW_INTEGRATION_TIME
        + n_system * HW_SYSTEM_TIME
    )

    hw_flashing_time = (
        n_unit * HW_UNIT_FLASH_TIME
        + n_integration * HW_INTEGRATION_FLASH_TIME
        + n_system * HW_SYSTEM_FLASH_TIME
    )

    hw_total = (
        BUILD_TIME_HW
        + hw_flashing_time
        + hw_execution_time
    )
    # --------------------------------------------------------
    # Throughput
    #
    # Q = N_tests / T_total
    # --------------------------------------------------------

    native_total_minutes = native_total / 60.0
    hw_total_minutes = hw_total / 60.0

    q_native = n_tests / native_total_minutes
    q_hw = n_tests / hw_total_minutes

    # Speed-up in total turnover time
    s = hw_total / native_total

    native_total_times.append(native_total_minutes)
    hw_total_times.append(hw_total_minutes)

    native_throughput.append(q_native)
    hw_throughput.append(q_hw)

    speedup.append(s)


# Convert to numpy arrays
native_total_times = np.array(native_total_times)
hw_total_times = np.array(hw_total_times)

native_throughput = np.array(native_throughput)
hw_throughput = np.array(hw_throughput)

speedup = np.array(speedup)


# ============================================================
# Console summary
# ============================================================

print(
    f"{'Tests':>8} "
    f"{'Host exec + sim [min]':>24} "
    f"{'Hardware exec [min]':>22} "
    f"{'Q host [test/min]':>20} "
    f"{'Q hardware [test/min]':>24} "
    f"{'Speed-up':>12}"
)

for i in range(len(test_counts)):
    print(
        f"{test_counts[i]:8d} "
        f"{native_total_times[i]:14.2f} "
        f"{hw_total_times[i]:14.2f} "
        f"{native_throughput[i]:20.4f} "
        f"{hw_throughput[i]:16.4f} "
        f"{speedup[i]:12.2f}"
    )


# ============================================================
# Figure 1: Total verification turnover time
# ============================================================

plt.figure(figsize=(7, 4.5))

plt.plot(
    test_counts,
    hw_total_times,
    marker="o",
    markersize=3,
    label="Hardware execution"
)

plt.plot(
    test_counts,
    native_total_times,
    marker="s",
    markersize=3,
    label="Host execution + simulation"
)

plt.xlabel("Number of tests")
plt.ylabel("Total execution time [min]")
plt.grid(True, alpha=0.25)
plt.legend()
plt.tight_layout()

plt.savefig(
    "execution_turnover_time.pdf",
    bbox_inches="tight"
)

plt.savefig(
    "execution_turnover_time.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()


# ============================================================
# Figure 2: Test throughput
# ============================================================

plt.figure(figsize=(7, 4.5))

plt.plot(
    test_counts,
    native_throughput,
    marker="s",
    markersize=3,
    label="Host execution + simulation"
)

plt.plot(
    test_counts,
    hw_throughput,
    marker="o",
    markersize=3,
    label="Hardware execution"
)

plt.xlabel("Number of tests")
plt.ylabel("Throughput [tests/min]")
plt.grid(True, alpha=0.25)
plt.legend()
plt.tight_layout()

plt.savefig(
    "test_throughput.pdf",
    bbox_inches="tight"
)

plt.savefig(
    "test_throughput.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()


# ============================================================
# Figure 3: Native execution speed-up
# ============================================================

plt.figure(figsize=(7, 4.5))

plt.plot(
    test_counts,
    speedup,
    marker="o",
    markersize=3
)

plt.xlabel("Number of tests")
plt.ylabel("Speed-up relative to hardware [$\\times$]")
plt.grid(True, alpha=0.25)
plt.tight_layout()

plt.savefig(
    "native_speedup.pdf",
    bbox_inches="tight"
)

plt.savefig(
    "native_speedup.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()
