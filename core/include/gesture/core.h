/*
 * Gesture recognition core: C ABI.
 *
 * Consumed by fw_recognize (MCU) and by host/crates/gesture-core (Rust FFI).
 * Pure: no I/O, no clock, no global state; the caller owns all state buffers.
 * Inputs are raw LSM6DS3 LSB samples with MCU timestamps.
 */
#ifndef GESTURE_CORE_H
#define GESTURE_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TODO: state struct, init/feed/result API. Keep in sync with gesture-core/src/ffi.rs. */

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_CORE_H */
