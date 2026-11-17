// In-memory model of the ESP-IDF 4.4 NVS partition (what nvs.h operates on),
// plus test helpers to wipe, seed, dump, fault-inject and power-cut it.
//
// Behaviour modelled on IDF 4.4.7 components/nvs_flash (nvs_storage.cpp,
// nvs_page.cpp, nvs_api.cpp, nvs_handle_simple.cpp):
// - Namespaces are created by nvs_open(READWRITE); READONLY open of a missing
//   namespace returns ESP_ERR_NVS_NOT_FOUND. Names and keys are max 15 chars;
//   creating/writing a longer one returns ESP_ERR_NVS_KEY_TOO_LONG, lookups
//   compare only the first 15 chars (strncmp(.., 15) in IDF).
// - An entry is identified by namespace + key + type. A get with a different
//   type returns ESP_ERR_NVS_NOT_FOUND and leaves *out untouched. A set with a
//   different type adds a second entry (4.4 writeItem() looks up by type; the
//   "replace on type change" behaviour only arrived in IDF 5.x).
//   Simplification: on real flash a same-key entry of another type that sits
//   earlier on the same page can shadow the new one (Page::findItem returns
//   TYPE_MISMATCH); this model always finds the matching type.
// - Writes are visible immediately to every handle. nvs_commit() is a no-op
//   that only validates the handle (NVSHandleSimple::commit returns ESP_OK).
// - nvs_erase_all() removes the entries of that namespace only; the namespace
//   itself keeps existing (its index entry lives in namespace 0).
// - nvs_flash_erase() wipes everything and de-initializes; nvs_open() then
//   fails with ESP_ERR_NVS_NOT_INITIALIZED until nvs_flash_init().
//
// Every flash-mutating call (namespace creation, nvs_set_*, nvs_erase_key,
// nvs_erase_all) is a numbered "mutating op". emuNvsArmPowerCut(n) throws
// emu_nvs::PowerCut before the n-th op from now, leaving flash exactly as it
// was after the previous op (NVS entry writes are atomic).
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "esp_err.h"

// Error codes, values as in IDF 4.4 nvs.h.
#define ESP_ERR_NVS_BASE 0x1100
#define ESP_ERR_NVS_NOT_INITIALIZED (ESP_ERR_NVS_BASE + 0x01)
#define ESP_ERR_NVS_NOT_FOUND (ESP_ERR_NVS_BASE + 0x02)
#define ESP_ERR_NVS_TYPE_MISMATCH (ESP_ERR_NVS_BASE + 0x03)
#define ESP_ERR_NVS_READ_ONLY (ESP_ERR_NVS_BASE + 0x04)
#define ESP_ERR_NVS_NOT_ENOUGH_SPACE (ESP_ERR_NVS_BASE + 0x05)
#define ESP_ERR_NVS_INVALID_NAME (ESP_ERR_NVS_BASE + 0x06)
#define ESP_ERR_NVS_INVALID_HANDLE (ESP_ERR_NVS_BASE + 0x07)
#define ESP_ERR_NVS_REMOVE_FAILED (ESP_ERR_NVS_BASE + 0x08)
#define ESP_ERR_NVS_KEY_TOO_LONG (ESP_ERR_NVS_BASE + 0x09)
#define ESP_ERR_NVS_PAGE_FULL (ESP_ERR_NVS_BASE + 0x0a)
#define ESP_ERR_NVS_INVALID_STATE (ESP_ERR_NVS_BASE + 0x0b)
#define ESP_ERR_NVS_INVALID_LENGTH (ESP_ERR_NVS_BASE + 0x0c)
#define ESP_ERR_NVS_NO_FREE_PAGES (ESP_ERR_NVS_BASE + 0x0d)
#define ESP_ERR_NVS_VALUE_TOO_LONG (ESP_ERR_NVS_BASE + 0x0e)
#define ESP_ERR_NVS_PART_NOT_FOUND (ESP_ERR_NVS_BASE + 0x0f)
#define ESP_ERR_NVS_NEW_VERSION_FOUND (ESP_ERR_NVS_BASE + 0x10)

namespace emu_nvs {

// Values are the IDF ItemType codes (nvs_types.hpp).
enum class Type : uint8_t {
  U8 = 0x01,
  I8 = 0x11,
  U16 = 0x02,
  I16 = 0x12,
  U32 = 0x04,
  I32 = 0x14,
  U64 = 0x08,
  I64 = 0x18,
  STR = 0x21,
  BLOB = 0x42,
};

inline const char* typeName(Type t) {
  switch (t) {
    case Type::U8: return "u8";
    case Type::I8: return "i8";
    case Type::U16: return "u16";
    case Type::I16: return "i16";
    case Type::U32: return "u32";
    case Type::I32: return "i32";
    case Type::U64: return "u64";
    case Type::I64: return "i64";
    case Type::STR: return "str";
    case Type::BLOB: return "blob";
  }
  return "?";
}

constexpr size_t kMaxKeyLength = 15;           // NVS_KEY_NAME_MAX_SIZE - 1
constexpr size_t kMaxStrLength = 4000;         // Page::CHUNK_MAX_SIZE
constexpr size_t kMaxBlobLength = 4000 * 4;    // 0x5000 partition: 5 pages - 1

struct Entry {
  std::string ns;
  std::string key;
  Type type;
  std::vector<uint8_t> data;

  bool operator==(const Entry& o) const {
    return ns == o.ns && key == o.key && type == o.type && data == o.data;
  }
};

struct NsStats {
  int opens = 0;
  int reads = 0;
  int writes = 0;
  int erases = 0;
  int commits = 0;
};

// Thrown by a mutating op when an armed power cut fires.
struct PowerCut {
  long opIndex;
  std::string op;
};

struct Handle {
  std::string ns;
  bool readOnly;
};

struct Op {
  std::string ns;
  std::string what;  // e.g. "set openppg/scr_rot:u8"
};

struct State {
  bool initialized = true;
  std::vector<std::string> namespaces;  // creation order
  std::vector<Entry> entries;           // order of last write
  std::map<uint32_t, Handle> handles;
  uint32_t nextHandle = 1;
  std::map<std::string, NsStats> stats;
  long mutatingOps = 0;
  long powerCutAt = -1;
  esp_err_t failWrites = ESP_OK;
  esp_err_t failOpen = ESP_OK;
  std::vector<Op> opLog;
};

inline State& state() {
  static State s;
  return s;
}

inline std::string clampName(const char* name) {
  return std::string(name).substr(0, kMaxKeyLength);
}

inline bool nsExists(const std::string& ns) {
  const State& s = state();
  return std::find(s.namespaces.begin(), s.namespaces.end(), ns) != s.namespaces.end();
}

inline Entry* find(const std::string& ns, const std::string& key, Type t) {
  for (Entry& e : state().entries) {
    if (e.ns == ns && e.key == key && e.type == t) return &e;
  }
  return nullptr;
}

inline void mutation(const std::string& ns, const std::string& what) {
  State& s = state();
  if (s.powerCutAt >= 0 && s.mutatingOps >= s.powerCutAt) {
    s.powerCutAt = -1;
    throw PowerCut{s.mutatingOps, what};
  }
  s.opLog.push_back(Op{ns, what});
  s.mutatingOps++;
}

inline void upsert(const std::string& ns, const std::string& key, Type t,
                   const void* data, size_t len) {
  State& s = state();
  for (auto it = s.entries.begin(); it != s.entries.end(); ++it) {
    if (it->ns == ns && it->key == key && it->type == t) {
      s.entries.erase(it);
      break;
    }
  }
  const uint8_t* p = static_cast<const uint8_t*>(data);
  s.entries.push_back(Entry{ns, key, t, std::vector<uint8_t>(p, p + len)});
}

inline Handle* handle(uint32_t h) {
  auto& hs = state().handles;
  auto it = hs.find(h);
  return it == hs.end() ? nullptr : &it->second;
}

inline esp_err_t open(const char* name, bool readOnly, uint32_t* out) {
  State& s = state();
  if (out == nullptr || name == nullptr) return ESP_ERR_INVALID_ARG;
  if (!s.initialized) return ESP_ERR_NVS_NOT_INITIALIZED;
  if (s.failOpen != ESP_OK) return s.failOpen;
  const std::string ns = clampName(name);
  if (!nsExists(ns)) {
    if (readOnly) return ESP_ERR_NVS_NOT_FOUND;
    if (strlen(name) > kMaxKeyLength) return ESP_ERR_NVS_KEY_TOO_LONG;
    if (s.failWrites != ESP_OK) return s.failWrites;
    mutation(ns, "create-ns " + ns);
    s.namespaces.push_back(ns);
  }
  s.stats[ns].opens++;
  const uint32_t h = s.nextHandle++;
  s.handles[h] = Handle{ns, readOnly};
  *out = h;
  return ESP_OK;
}

inline esp_err_t set(uint32_t h, const char* key, Type t, const void* data, size_t len) {
  State& s = state();
  Handle* hd = handle(h);
  if (hd == nullptr) return ESP_ERR_NVS_INVALID_HANDLE;
  if (hd->readOnly) return ESP_ERR_NVS_READ_ONLY;
  if (key == nullptr) return ESP_ERR_INVALID_ARG;
  if (strlen(key) > kMaxKeyLength) return ESP_ERR_NVS_KEY_TOO_LONG;
  if (t == Type::STR && len > kMaxStrLength) return ESP_ERR_NVS_VALUE_TOO_LONG;
  if (t == Type::BLOB && len > kMaxBlobLength) return ESP_ERR_NVS_VALUE_TOO_LONG;
  if (s.failWrites != ESP_OK) return s.failWrites;
  mutation(hd->ns, std::string("set ") + hd->ns + "/" + key + ":" + typeName(t));
  s.stats[hd->ns].writes++;
  upsert(hd->ns, key, t, data, len);
  return ESP_OK;
}

// Fixed-size integer get. *out is written only on success.
inline esp_err_t getFixed(uint32_t h, const char* key, Type t, void* out, size_t len) {
  Handle* hd = handle(h);
  if (hd == nullptr) return ESP_ERR_NVS_INVALID_HANDLE;
  if (key == nullptr || out == nullptr) return ESP_ERR_INVALID_ARG;
  state().stats[hd->ns].reads++;
  const Entry* e = find(hd->ns, clampName(key), t);
  if (e == nullptr) return ESP_ERR_NVS_NOT_FOUND;
  memcpy(out, e->data.data(), len);
  return ESP_OK;
}

// nvs_get_str_or_blob() semantics from nvs_api.cpp.
inline esp_err_t getVar(uint32_t h, const char* key, Type t, void* out, size_t* length) {
  Handle* hd = handle(h);
  if (hd == nullptr) return ESP_ERR_NVS_INVALID_HANDLE;
  if (key == nullptr) return ESP_ERR_INVALID_ARG;
  state().stats[hd->ns].reads++;
  const Entry* e = find(hd->ns, clampName(key), t);
  if (e == nullptr) return ESP_ERR_NVS_NOT_FOUND;
  const size_t size = e->data.size();
  if (length == nullptr) return ESP_ERR_NVS_INVALID_LENGTH;
  if (out == nullptr) {
    *length = size;
    return ESP_OK;
  }
  if (*length < size) {
    *length = size;
    return ESP_ERR_NVS_INVALID_LENGTH;
  }
  *length = size;
  memcpy(out, e->data.data(), size);
  return ESP_OK;
}

inline esp_err_t eraseKey(uint32_t h, const char* key) {
  State& s = state();
  Handle* hd = handle(h);
  if (hd == nullptr) return ESP_ERR_NVS_INVALID_HANDLE;
  if (hd->readOnly) return ESP_ERR_NVS_READ_ONLY;
  if (key == nullptr) return ESP_ERR_INVALID_ARG;
  const std::string k = clampName(key);
  for (auto it = s.entries.begin(); it != s.entries.end(); ++it) {
    if (it->ns == hd->ns && it->key == k) {
      mutation(hd->ns, "erase " + hd->ns + "/" + k);
      s.stats[hd->ns].erases++;
      s.entries.erase(it);
      return ESP_OK;
    }
  }
  return ESP_ERR_NVS_NOT_FOUND;
}

inline esp_err_t eraseAll(uint32_t h) {
  State& s = state();
  Handle* hd = handle(h);
  if (hd == nullptr) return ESP_ERR_NVS_INVALID_HANDLE;
  if (hd->readOnly) return ESP_ERR_NVS_READ_ONLY;
  if (s.failWrites != ESP_OK) return s.failWrites;
  const std::string ns = hd->ns;
  mutation(ns, "erase-all " + ns);
  s.stats[ns].erases++;
  s.entries.erase(std::remove_if(s.entries.begin(), s.entries.end(),
                                 [&](const Entry& e) { return e.ns == ns; }),
                  s.entries.end());
  return ESP_OK;
}

inline esp_err_t commit(uint32_t h) {
  Handle* hd = handle(h);
  if (hd == nullptr) return ESP_ERR_NVS_INVALID_HANDLE;
  state().stats[hd->ns].commits++;
  return ESP_OK;
}

inline void close(uint32_t h) { state().handles.erase(h); }

inline std::string formatValue(const Entry& e) {
  char buf[64];
  auto u = [&](size_t n) {
    uint64_t v = 0;
    memcpy(&v, e.data.data(), n);
    snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v));
    return std::string(buf);
  };
  switch (e.type) {
    case Type::U8: return u(1);
    case Type::U16: return u(2);
    case Type::U32: return u(4);
    case Type::U64: return u(8);
    case Type::I8: { int8_t v; memcpy(&v, e.data.data(), 1); return std::to_string(v); }
    case Type::I16: { int16_t v; memcpy(&v, e.data.data(), 2); return std::to_string(v); }
    case Type::I32: { int32_t v; memcpy(&v, e.data.data(), 4); return std::to_string(v); }
    case Type::I64: { int64_t v; memcpy(&v, e.data.data(), 8); return std::to_string(v); }
    case Type::STR:
      return "\"" + std::string(e.data.begin(), e.data.end() - (e.data.empty() ? 0 : 1)) + "\"";
    case Type::BLOB: {
      std::string out = "[";
      for (size_t i = 0; i < e.data.size(); i++) {
        snprintf(buf, sizeof(buf), "%s%02x", i ? " " : "", e.data[i]);
        out += buf;
      }
      return out + "]";
    }
  }
  return "?";
}

}  // namespace emu_nvs

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

// Factory-fresh chip: no namespaces, no entries, NVS initialized, no faults.
inline void emuNvsWipe() { emu_nvs::state() = emu_nvs::State(); }

// Whole-flash erase (esptool erase_flash / nvs_flash_erase): everything is
// gone and NVS must be re-initialized by the next boot.
inline void emuNvsEraseFlash() {
  emu_nvs::State& s = emu_nvs::state();
  s.namespaces.clear();
  s.entries.clear();
  s.handles.clear();
  s.initialized = false;
}

// Power cycle: RAM-side state (open handles, armed faults) is lost, flash
// contents survive. The next boot must call nvs_flash_init().
inline void emuNvsPowerCycle() {
  emu_nvs::State& s = emu_nvs::state();
  s.handles.clear();
  s.initialized = false;
  s.powerCutAt = -1;
  s.failWrites = ESP_OK;
  s.failOpen = ESP_OK;
}

inline void emuNvsResetStats() {
  emu_nvs::State& s = emu_nvs::state();
  s.stats.clear();
  s.mutatingOps = 0;
  s.opLog.clear();
}

inline emu_nvs::NsStats emuNvsStats(const std::string& ns) {
  auto& st = emu_nvs::state().stats;
  auto it = st.find(ns);
  return it == st.end() ? emu_nvs::NsStats() : it->second;
}

inline long emuNvsMutatingOps() { return emu_nvs::state().mutatingOps; }
inline const std::vector<emu_nvs::Op>& emuNvsOpLog() { return emu_nvs::state().opLog; }

// Namespaces that received at least one mutating op since emuNvsResetStats().
inline std::vector<std::string> emuNvsWrittenNamespaces() {
  std::vector<std::string> out;
  for (const auto& op : emu_nvs::state().opLog) {
    if (std::find(out.begin(), out.end(), op.ns) == out.end()) out.push_back(op.ns);
  }
  return out;
}
inline size_t emuNvsOpenHandleCount() { return emu_nvs::state().handles.size(); }

// Throw emu_nvs::PowerCut before the n-th mutating op from now (0 = the next).
inline void emuNvsArmPowerCut(long n) {
  emu_nvs::State& s = emu_nvs::state();
  s.powerCutAt = s.mutatingOps + n;
}
inline void emuNvsDisarmPowerCut() { emu_nvs::state().powerCutAt = -1; }

inline void emuNvsFailWrites(esp_err_t err) { emu_nvs::state().failWrites = err; }
inline void emuNvsFailOpen(esp_err_t err) { emu_nvs::state().failOpen = err; }

inline bool emuNvsNamespaceExists(const std::string& ns) { return emu_nvs::nsExists(ns); }

inline const emu_nvs::Entry* emuNvsGet(const std::string& ns, const std::string& key,
                                       emu_nvs::Type t) {
  return emu_nvs::find(ns, key, t);
}

inline bool emuNvsHasKey(const std::string& ns, const std::string& key) {
  for (const auto& e : emu_nvs::state().entries) {
    if (e.ns == ns && e.key == key) return true;
  }
  return false;
}

inline size_t emuNvsEntryCount(const std::string& ns) {
  size_t n = 0;
  for (const auto& e : emu_nvs::state().entries) {
    if (e.ns == ns) n++;
  }
  return n;
}

template <typename T>
inline bool emuNvsRead(const std::string& ns, const std::string& key, emu_nvs::Type t, T* out) {
  const emu_nvs::Entry* e = emu_nvs::find(ns, key, t);
  if (e == nullptr || e->data.size() != sizeof(T)) return false;
  memcpy(out, e->data.data(), sizeof(T));
  return true;
}

// Write straight to "flash", bypassing the API (for corrupt / foreign data).
inline void emuNvsSeed(const std::string& ns, const std::string& key, emu_nvs::Type t,
                       const void* data, size_t len) {
  if (!emu_nvs::nsExists(ns)) emu_nvs::state().namespaces.push_back(ns);
  emu_nvs::upsert(ns, key, t, data, len);
}

template <typename T>
inline void emuNvsSeedValue(const std::string& ns, const std::string& key, emu_nvs::Type t, T v) {
  emuNvsSeed(ns, key, t, &v, sizeof(v));
}

inline void emuNvsRemove(const std::string& ns, const std::string& key) {
  auto& es = emu_nvs::state().entries;
  es.erase(std::remove_if(es.begin(), es.end(),
                          [&](const emu_nvs::Entry& e) { return e.ns == ns && e.key == key; }),
           es.end());
}

// Sorted copy of every entry (optionally one namespace) for equality checks.
inline std::vector<emu_nvs::Entry> emuNvsSnapshot(const std::string& ns = "") {
  std::vector<emu_nvs::Entry> out;
  for (const auto& e : emu_nvs::state().entries) {
    if (ns.empty() || e.ns == ns) out.push_back(e);
  }
  std::sort(out.begin(), out.end(), [](const emu_nvs::Entry& a, const emu_nvs::Entry& b) {
    if (a.ns != b.ns) return a.ns < b.ns;
    if (a.key != b.key) return a.key < b.key;
    return a.type < b.type;
  });
  return out;
}

// Human-readable dump, one "ns/key:type = value" line per entry.
inline std::string emuNvsDump(const std::string& ns = "") {
  std::string out;
  for (const auto& e : emuNvsSnapshot(ns)) {
    out += e.ns + "/" + e.key + ":" + emu_nvs::typeName(e.type) + " = " +
           emu_nvs::formatValue(e) + "\n";
  }
  return out;
}
