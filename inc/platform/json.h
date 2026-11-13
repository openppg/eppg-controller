#pragma once
#include <cstdint>
#include <memory>
#include <type_traits>

#include "cJSON.h"

namespace platform::json {
struct Delete {
  void operator()(cJSON* value) const { cJSON_Delete(value); }
};
using Document = std::unique_ptr<cJSON, Delete>;
inline const cJSON* field(const cJSON* object, const char* key) { return cJSON_GetObjectItemCaseSensitive(object, key); }
inline bool has(const cJSON* object, const char* key) {
  const auto* value = field(object, key);
  return value && !cJSON_IsNull(value);
}
inline const char* string(const cJSON* object, const char* key, const char* fallback = "") {
  const auto* value = field(object, key);
  return cJSON_IsString(value) ? value->valuestring : fallback;
}
template <class T>
T get(const cJSON* object, const char* key, T fallback = {}) {
  const auto* value = field(object, key);
  if constexpr (std::is_same_v<T, bool>)
    return cJSON_IsBool(value) ? cJSON_IsTrue(value) : cJSON_IsNumber(value) ? value->valuedouble != 0 : fallback;
  else
    return cJSON_IsNumber(value) ? static_cast<T>(value->valuedouble) : fallback;
}
inline void set(cJSON* object, const char* key, const char* value) { cJSON_AddStringToObject(object, key, value); }
inline void set(cJSON* object, const char* key, bool value) { cJSON_AddBoolToObject(object, key, value); }
template <class T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
void set(cJSON* object, const char* key, T value) {
  cJSON_AddNumberToObject(object, key, static_cast<double>(value));
}
}  // namespace platform::json
