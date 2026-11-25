/* The dev bench's analog capture session: the glue between a study's analog
 * tap, the board's analog source backend, the marker input and the samples
 * port. */
#ifndef EMBARCH_ANALOG_CAPTURE_H_
#define EMBARCH_ANALOG_CAPTURE_H_

#include <stdint.h>

/* Starts a session at `sample_hz` on the study's `channel` index: backend
 * prepared, start header written to the samples port, sampling begun.
 * Returns 0 or a negative errno (-ENOTSUP where the board has no backend or
 * no samples port, -EINVAL for a rate or channel the board cannot do). */
int analog_capture_start(uint32_t sample_hz, uint8_t channel);

/* Ends the session and returns how many blocks it lost. Safe with no session. */
uint32_t analog_capture_stop(void);

#endif /* EMBARCH_ANALOG_CAPTURE_H_ */
