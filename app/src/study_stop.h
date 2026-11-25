/* Core's `StudyStop`, read while a study runs (schema v17, EMB-485).
 *
 * The dispatch loop is the only thing that runs a study and it blocks in the
 * BLE bridge and in step delays, so nothing else reads the link while one
 * runs. This module is how a stop reaches it: the dispatch thread polls it at
 * every point it can stop at, and each poll drains whatever the link has
 * received and latches a `StudyStop` when one has arrived. There is no second
 * thread and no lock; every call is from the dispatch thread.
 *
 * Pure logic with a byte source passed in, so a native_sim test drives it
 * without a UART.
 */
#ifndef EMBARCH_DEV_BENCH_STUDY_STOP_H_
#define EMBARCH_DEV_BENCH_STUDY_STOP_H_

#include <stdbool.h>
#include <stdint.h>

/* Returns 0 and one received byte, or a negative value when none is waiting. */
typedef int (*study_stop_byte_source)(uint8_t *byte);

/* Starts listening for the study about to run: forgets any earlier request
 * and any half-read frame, and reads from `source`. */
void study_stop_begin(study_stop_byte_source source);

/* Stops listening. A stop that arrives after this is left in the link's
 * buffer for the idle loop, which ignores one with no study running. */
void study_stop_end(void);

/* Drains the byte source and returns whether a `StudyStop` has been latched,
 * now or earlier in this study. Frames that are not a `StudyStop` are
 * discarded; Core sends nothing else while a study runs. */
bool study_stop_poll(void);

/* The reason (enum dbm_stop_reason) of the latched `StudyStop`. Meaningful
 * only once study_stop_poll() has returned true. */
uint8_t study_stop_reason(void);

/* Sleeps up to `ms`, polling for a stop between short slices. Returns true if
 * a stop is latched, at once or during the wait, and false if the full time
 * passed. */
bool study_stop_sleep_ms(uint32_t ms);

/* How long study_stop_sleep_ms() sleeps between polls, which bounds how late
 * a stop is noticed in a delay. */
#define STUDY_STOP_POLL_SLICE_MS 10U

#endif /* EMBARCH_DEV_BENCH_STUDY_STOP_H_ */
