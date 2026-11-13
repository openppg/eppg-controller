#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EPPG_STAGE_MAX_IMAGE (256u * 1024u)
#define EPPG_STAGE_SECTOR_SIZE 4096u
#define EPPG_STAGE_DATA_OFFSET EPPG_STAGE_SECTOR_SIZE

// Callbacks return zero on success. Offsets are relative to the partition.
typedef struct {
  void* context;
  uint32_t size;
  int (*read)(void*, uint32_t, void*, size_t);
  int (*write)(void*, uint32_t, const void*, size_t);
  int (*erase)(void*, uint32_t, size_t);
} eppg_stage_io;

typedef struct {
  eppg_stage_io io;
  uint32_t size;
  uint32_t received;
  bool active;
  bool sealed;
  // One sector is sufficient to preserve offset-addressed rewrite semantics
  // on NOR flash; the full 256 KiB image never needs an internal RAM buffer.
  uint8_t scratch[EPPG_STAGE_SECTOR_SIZE];
} eppg_image_store;

void eppg_image_store_init(eppg_image_store* store, const eppg_stage_io* io);
bool eppg_image_store_begin(eppg_image_store* store, uint32_t size);
int32_t eppg_image_store_write(eppg_image_store* store, uint32_t offset,
                             const uint8_t* bytes, size_t length);
bool eppg_image_store_read(eppg_image_store* store, uint32_t offset,
                          uint8_t* bytes, size_t length);
bool eppg_image_store_finish(eppg_image_store* store);
void eppg_image_store_abort(eppg_image_store* store);

#ifdef __cplusplus
}
#endif
