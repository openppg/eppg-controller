#include "image_staging.h"
#include <string.h>

// This ownership marker describes a reserved scratch area, not a filesystem.
// It is written once and never erased during subsequent transfers. A power
// loss during image erase/write therefore cannot lose ownership information.
static const uint8_t owner[16] = {
  'O','P','P','G','E','S','C','1', 1,0,0,0, 0x8a,0xed,0x42,0x95
};

void eppg_image_store_init(eppg_image_store* store, const eppg_stage_io* io) {
  memset(store, 0, sizeof(*store));
  if (io) store->io = *io;
}

static bool claim_unused_partition(eppg_image_store* store) {
  uint8_t found[sizeof(owner)];
  if (store->io.read(store->io.context, 0, found, sizeof(found))) return false;
  if (memcmp(found, owner, sizeof(owner)) == 0) return true;
  // Before first claim require every other byte of the partition to be erased.
  // Unknown existing filesystem/user data is never erased. Compatible partial
  // ownership programming can be completed after a power cut during first use.
  for (size_t i = 0; i < sizeof(owner); ++i) {
    if ((found[i] | owner[i]) != found[i]) return false;
  }
  for (uint32_t offset = sizeof(owner); offset < store->io.size;) {
    size_t length = store->io.size - offset;
    if (length > sizeof(store->scratch)) length = sizeof(store->scratch);
    if (store->io.read(store->io.context, offset, store->scratch, length)) return false;
    for (size_t i = 0; i < length; ++i) if (store->scratch[i] != 0xff) return false;
    offset += length;
  }
  return store->io.write(store->io.context, 0, owner, sizeof(owner)) == 0;
}

bool eppg_image_store_begin(eppg_image_store* store, uint32_t size) {
  if (!store || !store->io.read || !store->io.write || !store->io.erase ||
      store->active || !size || size > EPPG_STAGE_MAX_IMAGE ||
      store->io.size < EPPG_STAGE_DATA_OFFSET + EPPG_STAGE_MAX_IMAGE) return false;
  if (!claim_unused_partition(store)) return false;
  const size_t erase_size = (size + EPPG_STAGE_SECTOR_SIZE - 1) & ~(EPPG_STAGE_SECTOR_SIZE - 1);
  if (store->io.erase(store->io.context, EPPG_STAGE_DATA_OFFSET, erase_size)) return false;
  store->size = size;
  store->received = 0;
  store->active = true;
  store->sealed = false;
  return true;
}

int32_t eppg_image_store_write(eppg_image_store* store, uint32_t offset,
                             const uint8_t* bytes, size_t length) {
  if (!store || !store->active || store->sealed || (!bytes && length) ||
      offset > store->size || length > store->size - offset) return -1;
  const uint32_t start = offset;
  const uint32_t end = offset + length;
  while (length) {
    const uint32_t address = EPPG_STAGE_DATA_OFFSET + offset;
    const uint32_t sector = address & ~(EPPG_STAGE_SECTOR_SIZE - 1);
    const size_t within = address - sector;
    size_t count = EPPG_STAGE_SECTOR_SIZE - within;
    if (count > length) count = length;
    if (store->io.read(store->io.context, sector, store->scratch, sizeof(store->scratch))) return -1;
    bool erase = false;
    bool changed = false;
    for (size_t i = 0; i < count; ++i) {
      const uint8_t previous = store->scratch[within + i];
      changed |= previous != bytes[i];
      erase |= (previous & bytes[i]) != bytes[i];
    }
    if (changed) {
      if (erase) {
        memcpy(store->scratch + within, bytes, count);
        if (store->io.erase(store->io.context, sector, EPPG_STAGE_SECTOR_SIZE) ||
            store->io.write(store->io.context, sector, store->scratch, sizeof(store->scratch))) return -1;
      } else if (store->io.write(store->io.context, address, bytes, count)) return -1;
    }
    offset += count;
    bytes += count;
    length -= count;
  }
  // Match the legacy contiguous-from-zero accounting, including retries and
  // offset-addressed writes. A hole must be covered before FW_END is accepted.
  if (start <= store->received && end > store->received) store->received = end;
  return (int32_t)store->received;
}

bool eppg_image_store_read(eppg_image_store* store, uint32_t offset,
                          uint8_t* bytes, size_t length) {
  return store && store->active && (!length || bytes) && offset <= store->size &&
      length <= store->size - offset &&
      store->io.read(store->io.context, EPPG_STAGE_DATA_OFFSET + offset, bytes, length) == 0;
}
bool eppg_image_store_finish(eppg_image_store* store) {
  if (!store || !store->active || store->received != store->size) return false;
  store->sealed = true;
  return true;
}
void eppg_image_store_abort(eppg_image_store* store) {
  if (!store) return;
  store->active = store->sealed = false;
  store->received = store->size = 0;
}
