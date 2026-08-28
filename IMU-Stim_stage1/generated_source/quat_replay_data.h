#ifndef QUAT_REPLAY_DATA_H_
#define QUAT_REPLAY_DATA_H_

#include <stddef.h>

#include "sensors/sensors.h"

#define QUAT_REPLAY_RATE_HZ 60U

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
