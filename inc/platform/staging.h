#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// All functions return zero / nonnegative on success. Begin/write/finish/abort
// run on the maintenance owner while DISARMED. Read never waits on the mutex:
// -2 means retry next control tick; -1 is an invalid session or flash error.
int eppg_stage_begin(uint32_t size);
int32_t eppg_stage_write(uint32_t offset, const uint8_t* bytes, size_t length);
int eppg_stage_read(uint32_t offset, uint8_t* bytes, size_t length);
int eppg_stage_finish(void);
int eppg_stage_abort(void);

#ifdef __cplusplus
}
#endif
