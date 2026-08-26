#ifndef QUAT_REPLAY_DATA_H_
#define QUAT_REPLAY_DATA_H_

#include <stddef.h>

#define QUAT_REPLAY_RATE_HZ 60U

/*
 * SmartVNS quaternion representation.
 *
 * Component order:
 *     [x, y, z, w]
 */
struct quat_replay_sample
{
    float x;
    float y;
    float z;
    float w;
};

extern const struct quat_replay_sample quat_replay_data[];
extern const size_t quat_replay_data_count;

#endif /* QUAT_REPLAY_DATA_H_ */
