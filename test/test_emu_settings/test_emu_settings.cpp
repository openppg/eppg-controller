// Host emulation of settings storage (#127: device_settings reads moved from
// Arduino Preferences to raw nvs_*) and the first-boot factory QC gate (#118).
//
// The REAL firmware sources are compiled unmodified against
// test/emu_stubs/settings: an in-memory ESP-IDF 4.4 NVS (emu_nvs.h), the
// Arduino-ESP32 2.0.17 Preferences API on top of it (the 7.x/8.0 settings
// writer), and fakes for the QC screen, sensors, top button and throttle pot.
// Boots are emulated in main.cpp setup() order; see emuBootToGate().

#include <gtest/gtest.h>

#include <climits>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Arduino.h"
#include "Preferences.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "../../src/sp140/globals.cpp"
#include "../../src/sp140/esp32s3-config.cpp"
#include "../../src/sp140/device_settings.cpp"
#include "../../src/sp140/factory_settings.cpp"
#include "../../src/sp140/qc_logic.cpp"
#include "../../src/sp140/throttle.cpp"
#include "../../src/sp140/first_boot_qc.cpp"

using emu_nvs::Type;

static const char* const kUserNs = "openppg";
static const char* const kFactoryNs = "openppg-factory";
static const char* const kDiagNs = "openppg_diag";

// ---------------------------------------------------------------------------
// Emulated peripherals, bench operator and QC screen
// ---------------------------------------------------------------------------

struct EmuHardware {
  bool baroPresent = true;
  float baroHpa = 1001.5f;
  bool canDriversUp = true;  // TWAI driver + MCP2515 initialized on the board
  bool escPresent = true;    // ESC answering on CAN
  bool bmsPresent = true;    // BMS broadcasting on CAN
  float packV = 79.8f;
  float cpuC = 41.0f;
};

// Scripted operator: reacts to whatever QC prompt is on screen.
struct EmuOperator {
  uint16_t idleRaw = 150;
  uint16_t fullRaw = 3950;
  uint16_t recheckRaw = 160;
  bool squeezesForCues = true;   // confirms buzzer / vibe by squeezing
  bool pressesButton = true;     // "PRESS BUTTON" step
  bool confirmsSkips = true;     // "<dev> NOT DETECTED" skip-confirm
  std::string cutPowerAtPrompt;  // power dies the moment this prompt appears
};

struct EmuScreen {
  int setups = 0;
  int teardowns = 0;
  int banners = 0;
  bool bannerPassed = false;
  std::string bannerDetail;
  std::vector<std::string> prompts;
  std::vector<std::pair<std::string, QcCheckStatus>> checks;
  bool attemptedMarkedAtFlowStart = false;
};

struct Emu {
  EmuHardware hw;
  EmuOperator op;
  EmuScreen screen;
  std::string prompt;
  uint32_t promptStartMs = 0;
  int escThrottleCommands = 0;
};

static Emu g_emu;

// Globals the included sources expect from main.cpp / esc.cpp / bms.cpp.
HardwareConfig board_config;
bool bmsCanInitialized = false;
bool escTwaiInitialized = false;
bool bmpPresent = false;
STR_ESC_TELEMETRY_140 escTelemetryData;
STR_BMS_TELEMETRY_140 bmsTelemetryData;
const char* buildDate = "emu-build";
SemaphoreHandle_t lvglMutex = nullptr;
lv_obj_t* main_screen = nullptr;

bool sampleBarometer(const STR_DEVICE_DATA_140_V1&) { return g_emu.hw.baroPresent; }
float getBaroPressure() { return g_emu.hw.baroHpa; }
void requestEscHardwareInfo() {}
void readESCTelemetry() {
  if (!g_emu.hw.escPresent) return;
  escTelemetryData.escState = TelemetryState::CONNECTED;
  escTelemetryData.hardware_id = 0x0A21;
}
void updateBMSData() {
  if (!g_emu.hw.bmsPresent) return;
  bmsTelemetryData.bmsState = TelemetryState::CONNECTED;
  bmsTelemetryData.battery_voltage = g_emu.hw.packV;
  snprintf(bmsTelemetryData.battery_id, sizeof(bmsTelemetryData.battery_id), "EMU-BMS-0001");
}
void primeCpuTemperatureCache() {}
float getCachedCpuTemperature() { return g_emu.hw.cpuC; }
void startTone(uint16_t) {}
void stopTone() {}
void vibeDirectSet(uint8_t) {}
void setESCThrottle(int) { g_emu.escThrottleCommands++; }

void diagnosticsMarkPlannedRestart(PlannedRestartReason) {}
void diagnosticsSendJson(Stream&) {}
void diagnosticsClearPersistentData() {}

static int factoryU8(const char* key) {
  uint8_t v = 0;
  return emuNvsRead(kFactoryNs, key, Type::U8, &v) ? v : -1;
}

static int factoryU16(const char* key) {
  uint16_t v = 0;
  return emuNvsRead(kFactoryNs, key, Type::U16, &v) ? v : -1;
}

void setupQcScreen(bool) {
  g_emu.screen.setups++;
  g_emu.screen.attemptedMarkedAtFlowStart = factoryU8("qc_attempted") == 1;
}
void qcScreenSetCheck(uint8_t, const char* name, QcCheckStatus status, const char*) {
  g_emu.screen.checks.emplace_back(name, status);
}
void qcScreenPrompt(const char* instruction, const char*) {
  g_emu.prompt = instruction;
  g_emu.promptStartMs = millis();
  g_emu.screen.prompts.push_back(instruction);
  if (!g_emu.op.cutPowerAtPrompt.empty() && g_emu.op.cutPowerAtPrompt == instruction) {
    g_emu.op.cutPowerAtPrompt.clear();
    throw emu_nvs::PowerCut{-1, std::string("prompt ") + instruction};
  }
}
void qcScreenPromptValue(const char*) {}
void qcScreenPromptProgress(uint8_t) {}
void qcScreenShowChecklist() {}
void qcScreenBanner(bool passed, const char* detail) {
  g_emu.screen.banners++;
  g_emu.screen.bannerPassed = passed;
  g_emu.screen.bannerDetail = detail;
}
void teardownQcScreen(lv_obj_t*) { g_emu.screen.teardowns++; }

static bool promptStartsWith(const char* prefix) { return g_emu.prompt.rfind(prefix, 0) == 0; }

static uint16_t operatorThrottleRaw() {
  const EmuOperator& op = g_emu.op;
  if (g_emu.prompt == "SQUEEZE FULL") return op.fullRaw;
  if (g_emu.prompt == "RELEASE AGAIN") return op.recheckRaw;
  if (op.squeezesForCues && (promptStartsWith("TONE PLAYING") || promptStartsWith("VIBRATING"))) {
    return op.fullRaw;
  }
  return op.idleRaw;
}

// Top button (INPUT_PULLUP): a 200 ms press shortly after the prompt appears.
static int operatorButton() {
  const bool wanted =
      (g_emu.prompt == "PRESS BUTTON" && g_emu.op.pressesButton) ||
      (g_emu.prompt.find("NOT DETECTED") != std::string::npos && g_emu.op.confirmsSkips);
  if (!wanted) return HIGH;
  const uint32_t t = millis() - g_emu.promptStartMs;
  return (t >= 200 && t < 400) ? LOW : HIGH;
}

// ---------------------------------------------------------------------------
// Boot emulation
// ---------------------------------------------------------------------------

// Stand-in for diagnosticsInit() (compiled out of native builds): same
// namespace, written through Preferences on every boot like the real one.
static void emuDiagnosticsInit() {
  struct {
    uint32_t magic;
    uint32_t boot_counter;
    uint8_t next_slot;
    uint8_t count;
    uint16_t reserved;
  } meta = {0x44494147UL, 0, 0, 0, 0};
  Preferences p;
  if (!p.begin(kDiagNs, false)) return;
  p.getBytes("meta", &meta, sizeof(meta));
  meta.boot_counter++;
  p.putUChar("planned", 0);
  p.putBytes("meta", &meta, sizeof(meta));
  p.end();
}

// Power-on reset: RAM (including the included firmware's file statics) back
// to reset values, flash kept, then initArduino()'s nvs_flash_init().
static void emuPowerOn() {
  emuNvsPowerCycle();
  factoryPrefs().end();
  s_prefsMutex = nullptr;
  s_factoryMutex = nullptr;
  s_userSettingsPresentAtBoot = false;
  s_factoryQcPassedAtBoot = false;
  s_factoryQcAttemptedAtBoot = false;
  s_rerunRequestedAtBoot = false;
  s_contextCaptured = false;
  s_checkRow = 0;
  gWebSerialCommandLength = 0;
  gWebSerialCommandOverflow = false;
  deviceData = STR_DEVICE_DATA_140_V1();
  escTelemetryData = STR_ESC_TELEMETRY_140();
  bmsTelemetryData = STR_BMS_TELEMETRY_140();
  board_config = HardwareConfig();
  bmpPresent = bmsCanInitialized = escTwaiInitialized = false;
  lvglMutex = nullptr;
  throttleFilterClear();
  last_pot_raw = 0;
  USBSerial.reset();
  ESP.resetRestartCount();
  emuSetMillis(0);
  g_emu.screen = EmuScreen();
  g_emu.prompt.clear();
  nvs_flash_init();
}

// main.cpp setup() up to the Phase 4.5 gate, keeping every step that touches
// NVS or the gate, in order: diagnosticsInit -> qcCaptureBootContext ->
// refreshDeviceData -> createAllSyncPrimitives -> loadHardwareConfig /
// hardware init -> qcShouldRun. MainCppBootOrderMatchesEmulation pins this
// order to main.cpp. cutBeforeOp >= 0 kills power before that NVS write.
static bool emuBootToGate(long cutBeforeOp = -1) {
  emuPowerOn();
  if (cutBeforeOp >= 0) emuNvsArmPowerCut(cutBeforeOp);
  emuDiagnosticsInit();
  qcCaptureBootContext();
  refreshDeviceData();
  lvglMutex = xSemaphoreCreateMutex();
  board_config = s3_config;
  bmpPresent = g_emu.hw.baroPresent;
  bmsCanInitialized = g_emu.hw.canDriversUp;
  escTwaiInitialized = g_emu.hw.canDriversUp;
  return qcShouldRun();
}

struct BootResult {
  bool powerCut = false;
  bool qcRan = false;
};

static BootResult emuBoot(long cutBeforeOp = -1) {
  BootResult r;
  try {
    r.qcRan = emuBootToGate(cutBeforeOp);
    if (r.qcRan) runFirstBootQc();
  } catch (const emu_nvs::PowerCut&) {
    r.powerCut = true;
  }
  emuNvsDisarmPowerCut();
  return r;
}

static bool serialSaid(const char* text) {
  return USBSerial.output().find(text) != std::string::npos;
}

// ---------------------------------------------------------------------------
// Settings helpers
// ---------------------------------------------------------------------------

static STR_DEVICE_DATA_140_V1 makeSettings(uint8_t verMajor, uint8_t verMinor, uint8_t rotation,
                                           float seaPressure, bool metricTemp, bool metricAlt,
                                           uint8_t perfMode, uint8_t theme, uint16_t armedTime,
                                           uint8_t revision, int32_t tzOffset) {
  STR_DEVICE_DATA_140_V1 d = {};
  d.version_major = verMajor;
  d.version_minor = verMinor;
  d.screen_rotation = rotation;
  d.sea_pressure = seaPressure;
  d.metric_temp = metricTemp;
  d.metric_alt = metricAlt;
  d.performance_mode = perfMode;
  d.theme = theme;
  d.armed_time = armedTime;
  d.revision = revision;
  d.timezone_offset = tzOffset;
  return d;
}

// resetDeviceData() defaults for this firmware.
static STR_DEVICE_DATA_140_V1 defaultSettings() {
  return makeSettings(VERSION_MAJOR, VERSION_MINOR, 3, 1013.25f, true, true, 0, 0, 0, 0, 0);
}

static const STR_DEVICE_DATA_140_V1 kAllNonDefault =
    makeSettings(8, 0, 1, 1002.5f, false, false, 1, 1, 1234, 2, -18000);

// v8.0 writeDeviceData() body, line for line
// (git show v8.0:src/sp140/device_settings.cpp, lines 159-169).
static void writeSettingsAs80(const STR_DEVICE_DATA_140_V1& d) {
  Preferences preferences;
  ASSERT_TRUE(preferences.begin("openppg", false));
  preferences.putUChar("ver_major", d.version_major);
  preferences.putUChar("ver_minor", d.version_minor);
  preferences.putUChar("scr_rot", d.screen_rotation);
  preferences.putFloat("sea_pres", d.sea_pressure);
  preferences.putBool("metric_tmp", d.metric_temp);
  preferences.putBool("metric_alt", d.metric_alt);
  preferences.putUChar("perf_mode", d.performance_mode);
  preferences.putUChar("theme", d.theme);
  preferences.putUShort("armed_time", d.armed_time);
  preferences.putUChar("revision", d.revision);
  preferences.putInt("tz_offset", d.timezone_offset);
  preferences.end();
}

// v8.0 refreshDeviceData() getters (same file, lines 114-124), defaults as 8.0.
static STR_DEVICE_DATA_140_V1 readSettingsAs80() {
  STR_DEVICE_DATA_140_V1 d = {};
  Preferences preferences;
  if (!preferences.begin("openppg", false)) {
    ADD_FAILURE() << "8.0 reader could not open the namespace";
    return d;
  }
  d.version_major = preferences.getUChar("ver_major", 8);
  d.version_minor = preferences.getUChar("ver_minor", 0);
  d.screen_rotation = preferences.getUChar("scr_rot", 3);
  d.sea_pressure = preferences.getFloat("sea_pres", 1013.25);
  d.metric_temp = preferences.getBool("metric_tmp", true);
  d.metric_alt = preferences.getBool("metric_alt", true);
  d.performance_mode = preferences.getUChar("perf_mode", 0);
  d.theme = preferences.getUChar("theme", 0);
  d.armed_time = preferences.getUShort("armed_time", 0);
  d.revision = preferences.getUChar("revision", 0);
  d.timezone_offset = preferences.getInt("tz_offset", 0);
  preferences.end();
  return d;
}

static uint32_t floatBits(float f) {
  uint32_t b;
  memcpy(&b, &f, sizeof(b));
  return b;
}

// Field-by-field compare; sea_pressure bit-exact.
static ::testing::AssertionResult SameSettings(const STR_DEVICE_DATA_140_V1& expected,
                                               const STR_DEVICE_DATA_140_V1& actual) {
  std::ostringstream diff;
  auto cmp = [&](const char* name, long long e, long long a) {
    if (e != a) diff << "\n  " << name << ": expected " << e << ", got " << a;
  };
  cmp("version_major", expected.version_major, actual.version_major);
  cmp("version_minor", expected.version_minor, actual.version_minor);
  cmp("screen_rotation", expected.screen_rotation, actual.screen_rotation);
  cmp("metric_temp", expected.metric_temp, actual.metric_temp);
  cmp("metric_alt", expected.metric_alt, actual.metric_alt);
  cmp("performance_mode", expected.performance_mode, actual.performance_mode);
  cmp("theme", expected.theme, actual.theme);
  cmp("armed_time", expected.armed_time, actual.armed_time);
  cmp("revision", expected.revision, actual.revision);
  cmp("timezone_offset", expected.timezone_offset, actual.timezone_offset);
  const float es = expected.sea_pressure;
  const float as = actual.sea_pressure;
  if (floatBits(es) != floatBits(as)) {
    diff << "\n  sea_pressure: expected " << es << ", got " << as;
  }
  if (diff.str().empty()) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "settings differ:" << diff.str();
}

static ::testing::AssertionResult SameEntries(const std::vector<emu_nvs::Entry>& expected,
                                              const std::vector<emu_nvs::Entry>& actual) {
  if (expected == actual) return ::testing::AssertionSuccess();
  auto render = [](const std::vector<emu_nvs::Entry>& es) {
    std::string out;
    for (const auto& e : es) {
      out += "  " + e.ns + "/" + e.key + ":" + emu_nvs::typeName(e.type) + " = " +
             emu_nvs::formatValue(e) + "\n";
    }
    return out;
  };
  return ::testing::AssertionFailure() << "NVS entries differ\nexpected:\n"
                                       << render(expected) << "actual:\n" << render(actual);
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class EmuFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    emuNvsWipe();
    emuRtosStats() = EmuRtosStats();
    g_emu = Emu();
    emuAnalogReadHook() = [](uint8_t pin) -> uint16_t {
      return pin == board_config.throttle_pin ? operatorThrottleRaw() : 0;
    };
    emuDigitalReadHook() = [](uint8_t pin) -> int {
      return pin == board_config.button_top ? operatorButton() : HIGH;
    };
    emuPowerOn();
  }

  void TearDown() override {
    EXPECT_EQ(emuRtosStats().selfDeadlocks, 0) << "a mutex was re-taken by its holder";
    EXPECT_EQ(emuRtosStats().unbalancedGives, 0);
    EXPECT_EQ(emuNvsOpenHandleCount(), 0u) << "an NVS handle was left open";
    for (SemaphoreHandle_t m : {s_prefsMutex, s_factoryMutex, lvglMutex}) {
      if (m != nullptr) EXPECT_FALSE(m->held) << "a mutex was left held";
    }
    emuAnalogReadHook() = nullptr;
    emuDigitalReadHook() = nullptr;
  }
};

class SettingsEmu : public EmuFixture {};
class QcGateEmu : public EmuFixture {};
class ThrottleCalEmu : public EmuFixture {};

// ===========================================================================
// Settings storage (#127)
// ===========================================================================

struct SettingsCase {
  const char* name;
  STR_DEVICE_DATA_140_V1 d;
};

static const SettingsCase kSettingsCases[] = {
    {"AllNonDefault", kAllNonDefault},
    {"NegativeTzMaxArmedTime", makeSettings(8, 0, 1, 1013.0f, false, false, 1, 1, 65535, 1, -43200)},
    {"Rotation3LightChillPositiveTz", makeSettings(7, 6, 3, 950.25f, true, false, 0, 0, 1, 3, 50400)},
    {"Rotation1DarkSportHalfHourTz", makeSettings(8, 0, 1, 1040.0f, false, true, 1, 1, 600, 0, 19800)},
    {"PressureAtLowerLimit", makeSettings(8, 0, 3, 300.0f, false, false, 0, 1, 42, 0, -1)},
    {"PressureAtUpperLimit", makeSettings(8, 0, 1, 1200.0f, true, true, 1, 0, 9999, 255, 1)},
    {"TzInt32Min", makeSettings(8, 0, 1, 999.9f, false, false, 1, 1, 65535, 7, INT32_MIN)},
    {"TzInt32Max", makeSettings(8, 0, 3, 1013.25f, true, false, 0, 1, 0, 0, INT32_MAX)},
};

class SettingsEmuValues : public EmuFixture,
                          public ::testing::WithParamInterface<SettingsCase> {};

INSTANTIATE_TEST_SUITE_P(Sets, SettingsEmuValues, ::testing::ValuesIn(kSettingsCases),
                         [](const ::testing::TestParamInfo<SettingsCase>& info) {
                           return std::string(info.param.name);
                         });

// A unit that ran 8.0 boots 8.1: every field loads as 8.0 stored it and the
// load writes nothing (no reset, no "sanitize" rewrite).
TEST_P(SettingsEmuValues, Upgrade80PreservesAllFieldsAndWritesNothing) {
  const STR_DEVICE_DATA_140_V1 v = GetParam().d;
  writeSettingsAs80(v);
  emuDiagnosticsInit();
  const auto before = emuNvsSnapshot();

  emuPowerOn();
  emuNvsResetStats();
  refreshDeviceData();

  EXPECT_TRUE(SameSettings(v, deviceData));
  EXPECT_EQ(emuNvsMutatingOps(), 0) << emuNvsDump();
  EXPECT_TRUE(SameEntries(before, emuNvsSnapshot()));
  EXPECT_FALSE(serialSaid("No saved preferences"));
  EXPECT_FALSE(serialSaid("Sanitized"));
  EXPECT_TRUE(serialSaid("Device data loaded from Preferences"));
}

TEST_P(SettingsEmuValues, RoundTrip81WriteThenRefreshPreservesAllFields) {
  const STR_DEVICE_DATA_140_V1 v = GetParam().d;
  deviceData = v;
  writeDeviceData();

  emuPowerOn();
  emuNvsResetStats();
  refreshDeviceData();

  EXPECT_TRUE(SameSettings(v, deviceData));
  EXPECT_EQ(emuNvsMutatingOps(), 0);
}

// Downgrade 8.1 -> 8.0: the 8.0 Preferences getters read what 8.1 wrote.
TEST_P(SettingsEmuValues, Downgrade81To80ReaderSeesSameValues) {
  const STR_DEVICE_DATA_140_V1 v = GetParam().d;
  deviceData = v;
  writeDeviceData();

  emuPowerOn();
  EXPECT_TRUE(SameSettings(v, readSettingsAs80()));
}

// The 8.1 raw-NVS writer and the 8.0 Preferences writer produce byte-identical
// NVS entries (same keys, same types, same sizes, same bytes).
TEST_P(SettingsEmuValues, Writer81ProducesSameNvsEntriesAs80Writer) {
  const STR_DEVICE_DATA_140_V1 v = GetParam().d;
  writeSettingsAs80(v);
  const auto as80 = emuNvsSnapshot(kUserNs);

  emuNvsWipe();
  emuPowerOn();
  deviceData = v;
  writeDeviceData();

  EXPECT_TRUE(SameEntries(as80, emuNvsSnapshot(kUserNs)));
  EXPECT_EQ(emuNvsEntryCount(kUserNs), 11u);
}

// The key table in device_settings.cpp, as written on a fresh unit.
TEST_F(SettingsEmu, FreshNvsWritesDefaultsWithDocumentedKeyTypes) {
  refreshDeviceData();

  EXPECT_TRUE(serialSaid("No saved preferences found"));
  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
  uint8_t u8 = 0;
  ASSERT_TRUE(emuNvsRead(kUserNs, "ver_major", Type::U8, &u8));
  EXPECT_EQ(u8, VERSION_MAJOR);

  struct KeyType {
    const char* key;
    Type type;
    size_t size;
  };
  const KeyType table[] = {
      {"ver_major", Type::U8, 1},  {"ver_minor", Type::U8, 1},  {"scr_rot", Type::U8, 1},
      {"sea_pres", Type::BLOB, 4}, {"metric_tmp", Type::U8, 1}, {"metric_alt", Type::U8, 1},
      {"perf_mode", Type::U8, 1},  {"theme", Type::U8, 1},      {"armed_time", Type::U16, 2},
      {"revision", Type::U8, 1},   {"tz_offset", Type::I32, 4},
  };
  for (const auto& kt : table) {
    const emu_nvs::Entry* e = emuNvsGet(kUserNs, kt.key, kt.type);
    ASSERT_NE(e, nullptr) << kt.key << " missing or wrong type\n" << emuNvsDump();
    EXPECT_EQ(e->data.size(), kt.size) << kt.key;
  }
  EXPECT_EQ(emuNvsEntryCount(kUserNs), 11u);

  float sea = 0;
  ASSERT_TRUE(emuNvsRead(kUserNs, "sea_pres", Type::BLOB, &sea));
  EXPECT_EQ(floatBits(sea), floatBits(1013.25f));

  emuPowerOn();
  emuNvsResetStats();
  refreshDeviceData();
  EXPECT_EQ(emuNvsMutatingOps(), 0) << "second boot rewrote settings";
}

struct MissingKeyCase {
  const char* key;
  void (*toDefault)(STR_DEVICE_DATA_140_V1&);
};

static const MissingKeyCase kMissingKeyCases[] = {
    {"ver_minor", [](STR_DEVICE_DATA_140_V1& d) { d.version_minor = VERSION_MINOR; }},
    {"scr_rot", [](STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 3; }},
    {"sea_pres", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = 1013.25f; }},
    {"metric_tmp", [](STR_DEVICE_DATA_140_V1& d) { d.metric_temp = true; }},
    {"metric_alt", [](STR_DEVICE_DATA_140_V1& d) { d.metric_alt = true; }},
    {"perf_mode", [](STR_DEVICE_DATA_140_V1& d) { d.performance_mode = 0; }},
    {"theme", [](STR_DEVICE_DATA_140_V1& d) { d.theme = 0; }},
    {"armed_time", [](STR_DEVICE_DATA_140_V1& d) { d.armed_time = 0; }},
    {"revision", [](STR_DEVICE_DATA_140_V1& d) { d.revision = 0; }},
    {"tz_offset", [](STR_DEVICE_DATA_140_V1& d) { d.timezone_offset = 0; }},
};

class SettingsEmuMissingKey : public EmuFixture,
                              public ::testing::WithParamInterface<MissingKeyCase> {};

INSTANTIATE_TEST_SUITE_P(Keys, SettingsEmuMissingKey, ::testing::ValuesIn(kMissingKeyCases),
                         [](const ::testing::TestParamInfo<MissingKeyCase>& info) {
                           return std::string(info.param.key);
                         });

// One key absent: that field gets its documented default in RAM, every other
// field keeps its stored value, and nothing is written back.
TEST_P(SettingsEmuMissingKey, MissingKeyFallsBackToDefaultOthersKept) {
  writeSettingsAs80(kAllNonDefault);
  {
    Preferences p;
    ASSERT_TRUE(p.begin(kUserNs, false));
    ASSERT_TRUE(p.remove(GetParam().key));
    p.end();
  }
  STR_DEVICE_DATA_140_V1 expected = kAllNonDefault;
  GetParam().toDefault(expected);

  emuPowerOn();
  emuNvsResetStats();
  refreshDeviceData();

  EXPECT_TRUE(SameSettings(expected, deviceData));
  EXPECT_EQ(emuNvsMutatingOps(), 0);
  EXPECT_FALSE(emuNvsHasKey(kUserNs, GetParam().key));
}

// ver_major is the "settings exist" marker: without it the unit is treated as
// fresh and every stored value is replaced by defaults (same as 8.0).
TEST_F(SettingsEmu, MissingVerMajorIsTreatedAsFreshAndResetsEverything) {
  writeSettingsAs80(kAllNonDefault);
  emuNvsRemove(kUserNs, "ver_major");

  emuPowerOn();
  refreshDeviceData();

  EXPECT_TRUE(serialSaid("No saved preferences found"));
  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
  EXPECT_TRUE(SameSettings(defaultSettings(), readSettingsAs80()));
}

// Divergence from 8.0: 8.0 tested presence with Preferences.isKey() (any
// type); 8.1 requires a u8. A non-u8 ver_major now wipes the namespace.
// Every Preferences-era release wrote it with putUChar (u8), so no field unit
// has this layout; the test pins the behaviour.
TEST_F(SettingsEmu, VerMajorOfWrongTypeIsTreatedAsFresh_DiffersFrom80IsKey) {
  writeSettingsAs80(kAllNonDefault);
  emuNvsRemove(kUserNs, "ver_major");
  emuNvsSeedValue<uint16_t>(kUserNs, "ver_major", Type::U16, 8);

  emuPowerOn();
  {
    Preferences p;
    ASSERT_TRUE(p.begin(kUserNs, true));
    EXPECT_TRUE(p.isKey("ver_major")) << "8.0 (and the QC boot probe) would see settings";
    p.end();
  }
  refreshDeviceData();

  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
}

// sea_pres must be a 4-byte blob. Any other size, or a non-blob entry, loads
// as 1013.25 and leaves the other fields alone. (8.0's getFloat() would have
// copied the first 2 bytes of a 2-byte blob into the float.)
TEST_F(SettingsEmu, SeaPressureBlobOfWrongSizeOrTypeFallsBackToDefault) {
  struct Bad {
    const char* what;
    Type type;
    std::vector<uint8_t> bytes;
  };
  const double asDouble = 1002.5;
  std::vector<uint8_t> doubleBytes(sizeof(asDouble));
  memcpy(doubleBytes.data(), &asDouble, sizeof(asDouble));
  const Bad cases[] = {
      {"empty blob", Type::BLOB, {}},
      {"2-byte blob", Type::BLOB, {0x40, 0x7a}},
      {"3-byte blob", Type::BLOB, {0x00, 0xa0, 0x7a}},
      {"8-byte blob (double)", Type::BLOB, doubleBytes},
      {"u32 entry", Type::U32, {0x00, 0xa0, 0x7a, 0x44}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.what);
    emuNvsWipe();
    emuPowerOn();
    writeSettingsAs80(kAllNonDefault);
    emuNvsRemove(kUserNs, "sea_pres");
    emuNvsSeed(kUserNs, "sea_pres", c.type, c.bytes.data(), c.bytes.size());

    emuPowerOn();
    refreshDeviceData();

    STR_DEVICE_DATA_140_V1 expected = kAllNonDefault;
    expected.sea_pressure = 1013.25f;
    EXPECT_TRUE(SameSettings(expected, deviceData));
  }
}

// Lookup is by key AND type: an entry of the wrong type is invisible, so that
// field gets its default and the rest load normally.
TEST_F(SettingsEmu, WrongTypeEntryFallsBackToDefaultOthersKept) {
  struct Bad {
    const char* key;
    Type type;
    int64_t value;
    void (*toDefault)(STR_DEVICE_DATA_140_V1&);
  };
  const Bad cases[] = {
      {"ver_minor", Type::I8, 0, [](STR_DEVICE_DATA_140_V1& d) { d.version_minor = VERSION_MINOR; }},
      {"scr_rot", Type::I32, 1, [](STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 3; }},
      {"metric_tmp", Type::U16, 0, [](STR_DEVICE_DATA_140_V1& d) { d.metric_temp = true; }},
      {"metric_alt", Type::I8, 0, [](STR_DEVICE_DATA_140_V1& d) { d.metric_alt = true; }},
      {"perf_mode", Type::U16, 1, [](STR_DEVICE_DATA_140_V1& d) { d.performance_mode = 0; }},
      {"theme", Type::I8, 1, [](STR_DEVICE_DATA_140_V1& d) { d.theme = 0; }},
      {"armed_time", Type::U32, 500, [](STR_DEVICE_DATA_140_V1& d) { d.armed_time = 0; }},
      {"revision", Type::U16, 2, [](STR_DEVICE_DATA_140_V1& d) { d.revision = 0; }},
      {"tz_offset", Type::I16, -300, [](STR_DEVICE_DATA_140_V1& d) { d.timezone_offset = 0; }},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.key);
    emuNvsWipe();
    emuPowerOn();
    writeSettingsAs80(kAllNonDefault);
    emuNvsRemove(kUserNs, c.key);
    const size_t size = (c.type == Type::I8 || c.type == Type::U8)     ? 1
                        : (c.type == Type::I16 || c.type == Type::U16) ? 2
                                                                       : 4;
    emuNvsSeed(kUserNs, c.key, c.type, &c.value, size);

    emuPowerOn();
    refreshDeviceData();

    STR_DEVICE_DATA_140_V1 expected = kAllNonDefault;
    c.toDefault(expected);
    EXPECT_TRUE(SameSettings(expected, deviceData));
  }
}

// A u8 bool of 2 reads as false, exactly like 8.0's getBool() (== 1).
TEST_F(SettingsEmu, BoolStoredAsTwoReadsFalseSameAs80) {
  STR_DEVICE_DATA_140_V1 v = kAllNonDefault;
  v.metric_temp = true;
  v.metric_alt = true;
  writeSettingsAs80(v);
  emuNvsSeedValue<uint8_t>(kUserNs, "metric_tmp", Type::U8, 2);

  emuPowerOn();
  refreshDeviceData();

  EXPECT_FALSE(deviceData.metric_temp);
  EXPECT_TRUE(deviceData.metric_alt);
  EXPECT_FALSE(readSettingsAs80().metric_temp);
}

struct InvalidCase {
  const char* name;
  void (*corrupt)(STR_DEVICE_DATA_140_V1&);
  void (*sanitized)(STR_DEVICE_DATA_140_V1&);
};

static void rotTo3(STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 3; }
static void themeTo0(STR_DEVICE_DATA_140_V1& d) { d.theme = 0; }
static void perfTo0(STR_DEVICE_DATA_140_V1& d) { d.performance_mode = 0; }
static void seaToDefault(STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = 1013.25f; }

static const InvalidCase kInvalidCases[] = {
    {"Rotation0", [](STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 0; }, rotTo3},
    {"Rotation2", [](STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 2; }, rotTo3},
    {"Rotation4", [](STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 4; }, rotTo3},
    {"Rotation255", [](STR_DEVICE_DATA_140_V1& d) { d.screen_rotation = 255; }, rotTo3},
    {"Theme2", [](STR_DEVICE_DATA_140_V1& d) { d.theme = 2; }, themeTo0},
    {"Theme255", [](STR_DEVICE_DATA_140_V1& d) { d.theme = 255; }, themeTo0},
    {"PerfMode2", [](STR_DEVICE_DATA_140_V1& d) { d.performance_mode = 2; }, perfTo0},
    {"PerfMode255", [](STR_DEVICE_DATA_140_V1& d) { d.performance_mode = 255; }, perfTo0},
    {"PressureBelow300", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = 299.9f; }, seaToDefault},
    {"PressureAbove1200", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = 1200.1f; }, seaToDefault},
    {"PressureZero", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = 0.0f; }, seaToDefault},
    {"PressureNegative", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = -5.0f; }, seaToDefault},
    {"PressurePlusInfinity", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = INFINITY; }, seaToDefault},
    {"PressureMinusInfinity", [](STR_DEVICE_DATA_140_V1& d) { d.sea_pressure = -INFINITY; },
     seaToDefault},
};

class SettingsEmuInvalid : public EmuFixture, public ::testing::WithParamInterface<InvalidCase> {};

INSTANTIATE_TEST_SUITE_P(Values, SettingsEmuInvalid, ::testing::ValuesIn(kInvalidCases),
                         [](const ::testing::TestParamInfo<InvalidCase>& info) {
                           return std::string(info.param.name);
                         });

// An out-of-range stored value is replaced in RAM AND persisted with one
// 11-key write to the user namespace; the next boot loads it cleanly.
TEST_P(SettingsEmuInvalid, InvalidValueIsSanitizedAndPersisted) {
  STR_DEVICE_DATA_140_V1 stored = kAllNonDefault;
  GetParam().corrupt(stored);
  writeSettingsAs80(stored);
  STR_DEVICE_DATA_140_V1 expected = kAllNonDefault;
  GetParam().sanitized(expected);

  emuPowerOn();
  emuNvsResetStats();
  refreshDeviceData();

  EXPECT_TRUE(SameSettings(expected, deviceData));
  EXPECT_TRUE(serialSaid("Sanitized corrupted preference values"));
  EXPECT_EQ(emuNvsMutatingOps(), 11);
  EXPECT_EQ(emuNvsWrittenNamespaces(), std::vector<std::string>{kUserNs});
  EXPECT_TRUE(SameSettings(expected, readSettingsAs80()));

  emuPowerOn();
  emuNvsResetStats();
  refreshDeviceData();
  EXPECT_TRUE(SameSettings(expected, deviceData));
  EXPECT_EQ(emuNvsMutatingOps(), 0) << "sanitized value did not stick";
}

// Regression (pre-existing since 7.x, fixed in 8.1): the range check was
// `sea_pressure < 300 || sea_pressure > 1200`, which is false for NaN, so a
// NaN in NVS was loaded and kept and every altitude computed as NaN. The BLE
// sea-pressure write handler had the same comparison.
TEST_F(SettingsEmu, NanSeaPressureIsSanitizedToDefault) {
  STR_DEVICE_DATA_140_V1 v = kAllNonDefault;
  v.sea_pressure = NAN;
  writeSettingsAs80(v);

  emuPowerOn();
  refreshDeviceData();

  const float loaded = deviceData.sea_pressure;
  EXPECT_FALSE(std::isnan(loaded)) << "NaN sea_pressure survived refreshDeviceData()";
  EXPECT_EQ(floatBits(loaded), floatBits(1013.25f));
}

// User factory reset: defaults back, stale keys gone, and the factory QC and
// diagnostics namespaces are not touched.
TEST_F(SettingsEmu, FactoryResetRestoresDefaultsAndLeavesOtherNamespacesAlone) {
  writeSettingsAs80(kAllNonDefault);
  {
    Preferences p;
    ASSERT_TRUE(p.begin(kUserNs, false));
    p.putUChar("old_key", 9);
    p.end();
  }
  emuDiagnosticsInit();
  factoryMarkQcAttempted();
  factoryWriteCal(142, 3987);
  factoryWriteQcResult(true, factoryEncodeFw(8, 1));
  const char record[] = "{\"qc\":1}";
  ASSERT_TRUE(factoryWriteQcRecordBlob(record, sizeof(record)));
  const auto factoryBefore = emuNvsSnapshot(kFactoryNs);
  const auto diagBefore = emuNvsSnapshot(kDiagNs);

  emuPowerOn();
  refreshDeviceData();
  emuNvsResetStats();
  resetDeviceData();

  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
  EXPECT_TRUE(SameSettings(defaultSettings(), readSettingsAs80()));
  EXPECT_FALSE(emuNvsHasKey(kUserNs, "old_key"));
  EXPECT_EQ(emuNvsEntryCount(kUserNs), 11u);
  EXPECT_TRUE(SameEntries(factoryBefore, emuNvsSnapshot(kFactoryNs)));
  EXPECT_TRUE(SameEntries(diagBefore, emuNvsSnapshot(kDiagNs)));
  EXPECT_EQ(emuNvsWrittenNamespaces(), std::vector<std::string>{kUserNs});
}

TEST_F(SettingsEmu, NvsUnavailableFallsBackToDefaultsInRam) {
  emuNvsFailOpen(ESP_ERR_NVS_NOT_INITIALIZED);
  deviceData = kAllNonDefault;
  emuNvsResetStats();

  refreshDeviceData();

  EXPECT_TRUE(serialSaid("Failed to initialize Preferences"));
  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
  EXPECT_EQ(emuNvsMutatingOps(), 0);
  EXPECT_EQ(emuRtosStats().takes, emuRtosStats().gives);
}

TEST_F(SettingsEmu, NvsWriteFailureWarnsAndReleasesLock) {
  writeSettingsAs80(kAllNonDefault);
  emuPowerOn();
  refreshDeviceData();
  const auto before = emuNvsSnapshot(kUserNs);

  deviceData.theme = 0;
  emuNvsFailWrites(ESP_ERR_NVS_NOT_ENOUGH_SPACE);
  writeDeviceData();
  emuNvsFailWrites(ESP_OK);

  EXPECT_TRUE(serialSaid("Warning: Some preferences may not have been saved correctly"));
  EXPECT_TRUE(SameEntries(before, emuNvsSnapshot(kUserNs)));
  EXPECT_EQ(deviceData.theme, 0);
  ASSERT_NE(s_prefsMutex, nullptr);
  EXPECT_FALSE(s_prefsMutex->held);
}

// Web-serial "settings" JSON goes through sanitize + the same NVS writer.
TEST_F(SettingsEmu, SerialSettingsCommandSanitizesAndPersists) {
  refreshDeviceData();

  parse_serial_command_line(
      R"({"settings":{"screen_rot":1,"sea_pressure":1005.5,"metric_temp":false,)"
      R"("metric_alt":false,"performance_mode":1,"theme":1}})");
  STR_DEVICE_DATA_140_V1 expected = defaultSettings();
  expected.screen_rotation = 1;
  expected.sea_pressure = 1005.5f;
  expected.metric_temp = false;
  expected.metric_alt = false;
  expected.performance_mode = 1;
  expected.theme = 1;
  EXPECT_TRUE(SameSettings(expected, deviceData));
  EXPECT_TRUE(SameSettings(expected, readSettingsAs80()));
  EXPECT_TRUE(serialSaid("\"scr_rt\":1"));

  parse_serial_command_line(R"({"settings":{"screen_rot":2,"theme":7,"performance_mode":9}})");
  expected.screen_rotation = 3;
  expected.theme = 0;
  expected.performance_mode = 0;
  EXPECT_TRUE(SameSettings(expected, deviceData));
  EXPECT_TRUE(SameSettings(expected, readSettingsAs80()));
}

// ===========================================================================
// First-boot QC gate (#118), booting in main.cpp order
// ===========================================================================

// Fleet unit (8.0 settings, no factory namespace) boots 8.1: QC is skipped,
// qc_passed is back-filled without calibration, settings are untouched, and
// later boots skip QC without writing anything to the factory namespace.
TEST_F(QcGateEmu, FleetUnitUpgradingFrom80NeverEntersQc) {
  writeSettingsAs80(kAllNonDefault);
  emuDiagnosticsInit();
  const auto userBefore = emuNvsSnapshot(kUserNs);

  EXPECT_FALSE(emuBootToGate());
  EXPECT_TRUE(serialSaid("existing unit detected"));
  EXPECT_EQ(factoryU8("qc_passed"), 1);
  EXPECT_EQ(factoryU16("qc_fw"), factoryEncodeFw(VERSION_MAJOR, VERSION_MINOR));
  EXPECT_EQ(factoryU8("qc_attempted"), -1);
  EXPECT_EQ(factoryU8("pot_cal"), -1);
  EXPECT_TRUE(SameEntries(userBefore, emuNvsSnapshot(kUserNs)));
  EXPECT_TRUE(SameSettings(kAllNonDefault, deviceData));

  for (int boot = 2; boot <= 3; boot++) {
    SCOPED_TRACE("boot " + std::to_string(boot));
    emuNvsResetStats();
    const BootResult r = emuBoot();
    EXPECT_FALSE(r.qcRan);
    EXPECT_EQ(g_emu.screen.setups, 0);
    EXPECT_EQ(emuNvsStats(kFactoryNs).writes, 0);
    EXPECT_TRUE(SameSettings(kAllNonDefault, deviceData));
  }
}

// Power cut before every single NVS write of a fleet unit's first 8.1 boot:
// the next boot never enters QC and never loses a setting.
TEST_F(QcGateEmu, FleetUnitPowerCutAtEveryWriteOfFirstUpgradeBootNeverEntersQc) {
  writeSettingsAs80(kAllNonDefault);
  emuDiagnosticsInit();
  const emu_nvs::State flashBefore = emu_nvs::state();
  emuNvsResetStats();
  ASSERT_FALSE(emuBoot().qcRan);
  const std::vector<emu_nvs::Op> ops = emuNvsOpLog();
  ASSERT_FALSE(ops.empty());

  for (size_t k = 0; k < ops.size(); k++) {
    SCOPED_TRACE("power cut before op " + std::to_string(k) + " (" + ops[k].what + ")");
    emu_nvs::state() = flashBefore;
    emuNvsResetStats();
    ASSERT_TRUE(emuBoot(static_cast<long>(k)).powerCut);

    const BootResult next = emuBoot();
    EXPECT_FALSE(next.powerCut);
    EXPECT_FALSE(next.qcRan);
    EXPECT_EQ(g_emu.screen.setups, 0);
    EXPECT_TRUE(SameSettings(kAllNonDefault, deviceData));
    EXPECT_EQ(factoryU8("qc_passed"), 1);
  }
}

// Brand-new unit: QC runs, qc_attempted is persisted before any QC UI or
// hardware work, and once passed QC never runs again.
TEST_F(QcGateEmu, BrandNewUnitEntersQcMarksAttemptedFirstAndNeverAgainAfterPass) {
  ASSERT_TRUE(emuBootToGate());
  EXPECT_TRUE(serialSaid("fresh factory unit"));
  EXPECT_EQ(factoryU8("qc_attempted"), 1);
  EXPECT_EQ(factoryU8("qc_passed"), -1);
  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));

  runFirstBootQc();
  EXPECT_TRUE(g_emu.screen.attemptedMarkedAtFlowStart);
  EXPECT_EQ(g_emu.screen.banners, 1);
  EXPECT_TRUE(g_emu.screen.bannerPassed) << g_emu.screen.bannerDetail;
  EXPECT_EQ(factoryU8("qc_passed"), 1);
  EXPECT_EQ(factoryU16("qc_fw"), factoryEncodeFw(VERSION_MAJOR, VERSION_MINOR));
  EXPECT_EQ(factoryU8("pot_cal"), 1);

  for (int boot = 2; boot <= 4; boot++) {
    SCOPED_TRACE("boot " + std::to_string(boot));
    emuNvsResetStats();
    EXPECT_FALSE(emuBoot().qcRan);
    EXPECT_EQ(g_emu.screen.setups, 0);
    EXPECT_EQ(emuNvsStats(kFactoryNs).writes, 0);
  }
}

// Power dies while the operator is mid-calibration: no pass was recorded, so
// the next boot re-enters QC; after that pass it is skipped.
TEST_F(QcGateEmu, BrandNewUnitPowerCutMidQcRerunsQcNextBoot) {
  g_emu.op.cutPowerAtPrompt = "SQUEEZE FULL";
  const BootResult cut = emuBoot();
  ASSERT_TRUE(cut.powerCut);
  EXPECT_TRUE(cut.qcRan);
  EXPECT_EQ(factoryU8("qc_attempted"), 1);
  EXPECT_EQ(factoryU8("qc_passed"), -1);
  EXPECT_EQ(factoryU8("pot_cal"), -1);

  const BootResult retry = emuBoot();
  EXPECT_TRUE(retry.qcRan);
  EXPECT_TRUE(serialSaid("prior attempt incomplete"));
  EXPECT_TRUE(g_emu.screen.bannerPassed);
  EXPECT_EQ(factoryU8("qc_passed"), 1);

  EXPECT_FALSE(emuBoot().qcRan);
}

struct FreshTrace {
  emu_nvs::State flash;
  std::vector<emu_nvs::Op> ops;
  size_t verMajorOp = 0;   // index of the first ver_major write
  size_t attemptedOp = 0;  // index of the first qc_attempted write
};

static size_t findOp(const std::vector<emu_nvs::Op>& ops, const std::string& what) {
  for (size_t i = 0; i < ops.size(); i++) {
    if (ops[i].what == what) return i;
  }
  return ops.size();
}

// Every NVS write of a brand-new unit's first boot, through a passing QC.
static FreshTrace traceFreshBootWithQc() {
  FreshTrace t;
  emuNvsWipe();
  t.flash = emu_nvs::state();
  emuNvsResetStats();
  const BootResult r = emuBoot();
  EXPECT_TRUE(r.qcRan);
  EXPECT_EQ(factoryU8("qc_passed"), 1);
  t.ops = emuNvsOpLog();
  t.verMajorOp = findOp(t.ops, "set openppg/ver_major:u8");
  t.attemptedOp = findOp(t.ops, "set openppg-factory/qc_attempted:u8");
  // The attempt is on flash before any user setting, so no cut can leave a
  // fresh board looking like an upgraded fleet unit.
  EXPECT_LT(t.attemptedOp, t.verMajorOp);
  EXPECT_LT(t.verMajorOp, t.ops.size());
  return t;
}

// Cut before op k, reboot, and report if the unit ended up skipping QC
// without ever having passed it.
static std::string bootAfterCut(const FreshTrace& t, size_t k) {
  emu_nvs::state() = t.flash;
  emuNvsResetStats();
  if (!emuBoot(static_cast<long>(k)).powerCut) return "op " + std::to_string(k) + ": no cut";
  const bool passedBefore = factoryU8("qc_passed") == 1;
  const BootResult next = emuBoot();
  if (passedBefore && next.qcRan) {
    return "cut before op " + std::to_string(k) + " (" + t.ops[k].what + "): passed unit re-ran QC";
  }
  if (!passedBefore && !next.qcRan) {
    return "cut before op " + std::to_string(k) + " (" + t.ops[k].what +
           "): next boot skipped QC and back-filled qc_passed=" +
           std::to_string(factoryU8("qc_passed")) + " without a QC pass";
  }
  return "";
}

// Power cut before each NVS write from the qc_attempted mark through the end
// of QC: the unit either has a recorded pass (and skips) or runs QC again.
TEST_F(QcGateEmu, BrandNewUnitPowerCutAtEveryWriteAfterQcStartRerunsUntilPassed) {
  const FreshTrace t = traceFreshBootWithQc();
  std::string failures;
  for (size_t k = t.attemptedOp + 1; k < t.ops.size(); k++) {
    const std::string f = bootAfterCut(t, k);
    if (!f.empty()) failures += f + "\n";
  }
  EXPECT_TRUE(failures.empty()) << failures;
}

// Regression (#118 gate): on a brand-new unit, refreshDeviceData() writes
// ver_major in setup() Phase 1, and qc_attempted used to be written only by
// qcShouldRun() at Phase 4.5. A reset in between made the next boot see "user
// settings present, never attempted" and back-fill qc_passed=1, so the board
// never got QC. qcCaptureBootContext() now marks the attempt first, so a cut
// before any write of the first boot leads to QC on the next one.
TEST_F(QcGateEmu, BrandNewUnitPowerCutBeforeQcStartStillEntersQcNextBoot) {
  const FreshTrace t = traceFreshBootWithQc();
  std::string failures;
  for (size_t k = 0; k <= t.verMajorOp; k++) {
    const std::string f = bootAfterCut(t, k);
    if (!f.empty()) failures += f + "\n";
  }
  EXPECT_TRUE(failures.empty()) << failures;
}

// User factory reset (resetDeviceData) only clears the user namespace, so a
// QC-passed unit (or a back-filled fleet unit) keeps skipping QC.
TEST_F(QcGateEmu, UserFactoryResetOnQcPassedUnitDoesNotRunQc) {
  ASSERT_TRUE(emuBoot().qcRan);
  ASSERT_EQ(factoryU8("qc_passed"), 1);
  deviceData.theme = 1;
  writeDeviceData();
  const auto factoryBefore = emuNvsSnapshot(kFactoryNs);

  resetDeviceData();
  EXPECT_FALSE(emuBoot().qcRan);
  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
  EXPECT_TRUE(SameEntries(factoryBefore, emuNvsSnapshot(kFactoryNs)));
  const FactoryCal cal = factoryGetCal();
  EXPECT_TRUE(cal.calibrated);
  EXPECT_EQ(cal.potMin, 150);
  EXPECT_EQ(cal.potMax, 3950);

  emuNvsWipe();
  writeSettingsAs80(kAllNonDefault);
  ASSERT_FALSE(emuBoot().qcRan);
  resetDeviceData();
  EXPECT_FALSE(emuBoot().qcRan) << "back-filled fleet unit entered QC after a user reset";
}

// "run_qc" over web-serial: flag set + reboot; the next boot runs QC once and
// clears the flag before the flow starts; user settings are not touched.
TEST_F(QcGateEmu, RunQcSerialCommandRunsQcExactlyOnceAndClearsFlag) {
  writeSettingsAs80(kAllNonDefault);
  ASSERT_FALSE(emuBoot().qcRan);
  const auto userBefore = emuNvsSnapshot(kUserNs);

  USBSerial.feed("{\"command\":\"run_qc\"}\n");
  poll_serial_commands();
  EXPECT_EQ(ESP.restartCount(), 1);
  EXPECT_EQ(factoryU8("qc_rerun"), 1);

  ASSERT_TRUE(emuBootToGate());
  EXPECT_TRUE(serialSaid("rerun requested"));
  EXPECT_EQ(factoryU8("qc_rerun"), 0);
  runFirstBootQc();
  EXPECT_TRUE(g_emu.screen.bannerPassed);
  EXPECT_TRUE(SameEntries(userBefore, emuNvsSnapshot(kUserNs)));
  EXPECT_TRUE(SameSettings(kAllNonDefault, deviceData));

  for (int boot = 0; boot < 2; boot++) {
    EXPECT_FALSE(emuBoot().qcRan);
  }
}

// Documents current behaviour: a serial re-QC that FAILS stamps qc_passed=0,
// so the unit (even a customer unit) re-enters QC on every boot until a pass.
TEST_F(QcGateEmu, RunQcRerunThatFailsRetriesOnEveryBootUntilPass) {
  writeSettingsAs80(kAllNonDefault);
  ASSERT_FALSE(emuBoot().qcRan);
  USBSerial.feed("{\"command\":\"run_qc\"}\n");
  poll_serial_commands();

  g_emu.op.pressesButton = false;
  ASSERT_TRUE(emuBoot().qcRan);
  EXPECT_FALSE(g_emu.screen.bannerPassed);
  EXPECT_EQ(factoryU8("qc_passed"), 0);

  ASSERT_TRUE(emuBoot().qcRan);
  EXPECT_TRUE(serialSaid("prior attempt incomplete"));

  g_emu.op.pressesButton = true;
  ASSERT_TRUE(emuBoot().qcRan);
  EXPECT_TRUE(g_emu.screen.bannerPassed);
  EXPECT_FALSE(emuBoot().qcRan);
}

// Calibration rejected (full press only reaches 2900 < QC_MIN_FULL): nothing
// is saved, qc_passed=0, and the next boot runs QC again.
TEST_F(QcGateEmu, FreshUnitQcFailureSavesNoCalAndRetriesNextBoot) {
  g_emu.op.fullRaw = 2900;
  ASSERT_TRUE(emuBoot().qcRan);
  EXPECT_FALSE(g_emu.screen.bannerPassed);
  EXPECT_NE(g_emu.screen.bannerDetail.find("cal"), std::string::npos) << g_emu.screen.bannerDetail;
  EXPECT_EQ(factoryU8("qc_passed"), 0);
  EXPECT_EQ(factoryU8("pot_cal"), -1);

  g_emu.op.fullRaw = 3950;
  EXPECT_TRUE(emuBoot().qcRan);
  EXPECT_EQ(factoryU8("qc_passed"), 1);
}

// Documents current behaviour: erasing the whole flash of a customer unit
// (e.g. esptool erase_flash before a USB reflash) removes both namespaces, so
// the unit is indistinguishable from a new board and enters factory QC.
TEST_F(QcGateEmu, CustomerUnitAfterWholeFlashEraseEntersQc_CurrentBehaviour) {
  writeSettingsAs80(kAllNonDefault);
  ASSERT_FALSE(emuBoot().qcRan);
  ASSERT_EQ(factoryU8("qc_passed"), 1);

  emuNvsEraseFlash();
  const BootResult r = emuBoot();
  EXPECT_TRUE(r.qcRan);
  EXPECT_TRUE(serialSaid("fresh factory unit"));
  EXPECT_TRUE(SameSettings(defaultSettings(), deviceData));
}

TEST_F(QcGateEmu, GateFailsSafeToSkipWhenBootContextWasNotCaptured) {
  refreshDeviceData();
  emuNvsResetStats();
  EXPECT_FALSE(qcShouldRun());
  EXPECT_EQ(emuNvsMutatingOps(), 0);
  EXPECT_FALSE(emuNvsNamespaceExists(kFactoryNs));
}

// Why the probe must run before refreshDeviceData(): reversed, a brand-new
// board looks like a fleet unit and is back-filled as passed.
TEST_F(QcGateEmu, CaptureAfterSettingsLoadWouldMisclassifyFreshUnit) {
  refreshDeviceData();
  qcCaptureBootContext();
  EXPECT_FALSE(qcShouldRun());
  EXPECT_EQ(factoryU8("qc_passed"), 1);
}

static std::string readRepoFile(const std::string& rel) {
  std::vector<std::string> candidates;
  const std::string here = __FILE__;
  const size_t at = here.find("test/test_emu_settings/");
  if (at != std::string::npos) candidates.push_back(here.substr(0, at) + rel);
  candidates.push_back(rel);
  for (const auto& path : candidates) {
    std::ifstream f(path);
    if (f) {
      std::stringstream ss;
      ss << f.rdbuf();
      return ss.str();
    }
  }
  return "";
}

// Pins emuBootToGate() to the real setup() order in main.cpp.
TEST(QcGateEmuSource, MainCppBootOrderMatchesEmulation) {
  const std::string src = readRepoFile("src/sp140/main.cpp");
  ASSERT_FALSE(src.empty()) << "src/sp140/main.cpp not found";
  const size_t setup = src.find("void setup() {");
  ASSERT_NE(setup, std::string::npos);
  const char* const order[] = {"diagnosticsInit();", "qcCaptureBootContext();",
                               "refreshDeviceData();", "createAllSyncPrimitives();",
                               "loadHardwareConfig();", "if (qcShouldRun())",
                               "runFirstBootQc();"};
  size_t pos = setup;
  for (const char* step : order) {
    const size_t found = src.find(step, pos);
    ASSERT_NE(found, std::string::npos) << step << " missing or out of order in setup()";
    pos = found;
  }
}

// 8.1 QC'd unit -> 8.0 -> 8.1: settings survive both ways, QC stays skipped.
TEST_F(QcGateEmu, DowngradeTo80AndReupgradeKeepsSettingsAndSkipsQc) {
  ASSERT_TRUE(emuBoot().qcRan);
  STR_DEVICE_DATA_140_V1 v = kAllNonDefault;
  v.version_major = VERSION_MAJOR;
  v.version_minor = VERSION_MINOR;
  deviceData = v;
  writeDeviceData();

  emuPowerOn();
  EXPECT_TRUE(SameSettings(v, readSettingsAs80()));
  STR_DEVICE_DATA_140_V1 changedOn80 = v;
  changedOn80.theme = 0;
  changedOn80.armed_time = 1300;
  writeSettingsAs80(changedOn80);

  EXPECT_FALSE(emuBoot().qcRan);
  EXPECT_TRUE(SameSettings(changedOn80, deviceData));
}

// The QC flow writes only the factory namespace, leaves deviceData alone and
// never commands the ESC.
TEST_F(QcGateEmu, QcFlowWritesOnlyFactoryNamespaceAndNeverCommandsEsc) {
  ASSERT_TRUE(emuBootToGate());
  const auto userBefore = emuNvsSnapshot(kUserNs);
  const auto diagBefore = emuNvsSnapshot(kDiagNs);
  const STR_DEVICE_DATA_140_V1 ramBefore = deviceData;
  emuNvsResetStats();

  runFirstBootQc();

  EXPECT_TRUE(g_emu.screen.bannerPassed);
  EXPECT_EQ(emuNvsWrittenNamespaces(), std::vector<std::string>{kFactoryNs});
  EXPECT_TRUE(SameEntries(userBefore, emuNvsSnapshot(kUserNs)));
  EXPECT_TRUE(SameEntries(diagBefore, emuNvsSnapshot(kDiagNs)));
  EXPECT_TRUE(SameSettings(ramBefore, deviceData));
  EXPECT_EQ(g_emu.escThrottleCommands, 0);
}

TEST_F(QcGateEmu, QcRecordBlobIsReadableByBleSurface) {
  ASSERT_TRUE(emuBoot().qcRan);
  char buf[QC_RECORD_JSON_MAX];
  const size_t n = factoryReadQcRecordBlob(buf, sizeof(buf));
  ASSERT_GT(n, 0u);
  EXPECT_EQ(buf[n - 1], '\0');
  const std::string json(buf);
  EXPECT_NE(json.find("\"result\":\"PASSED\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"pot_min\":150"), std::string::npos) << json;
  EXPECT_NE(json.find("\"pot_max\":3950"), std::string::npos) << json;
  EXPECT_NE(json.find("\"cal_saved\":true"), std::string::npos) << json;

  char tiny[8];
  EXPECT_EQ(factoryReadQcRecordBlob(tiny, sizeof(tiny)), 0u);
}

// "openppg-factory" is exactly the 15-char NVS limit. IDF compares only the
// first 15 chars of a name, so a 16-char name either fails to create or
// silently aliases an existing namespace.
TEST_F(QcGateEmu, FactoryNamespaceAndKeysFitNvsLimits) {
  EXPECT_EQ(strlen(FACTORY_NAMESPACE), 15u);
  factoryMarkQcAttempted();
  EXPECT_TRUE(emuNvsNamespaceExists(kFactoryNs));
  nvs_handle_t h = 0;
  EXPECT_EQ(nvs_open("abcdefghijklmnop", NVS_READWRITE, &h), ESP_ERR_NVS_KEY_TOO_LONG);
  ASSERT_EQ(nvs_open("openppg-factory2", NVS_READONLY, &h), ESP_OK);
  uint8_t attempted = 0;
  EXPECT_EQ(nvs_get_u8(h, "qc_attempted", &attempted), ESP_OK);
  nvs_close(h);

  const char* const keys[] = {KEY_QC_PASSED, KEY_QC_ATTEMPTED, KEY_QC_FW,     KEY_POT_CALIBRATED,
                              KEY_POT_MIN,   KEY_POT_MAX,      KEY_QC_RERUN,  KEY_QC_RECORD,
                              KEY_NVS_SCRATCH, KEY_VERSION_MAJOR, KEY_VERSION_MINOR,
                              KEY_SCREEN_ROTATION, KEY_SEA_PRESSURE, KEY_METRIC_TEMP,
                              KEY_METRIC_ALT, KEY_PERFORMANCE_MODE, KEY_THEME, KEY_ARMED_TIME,
                              KEY_REVISION, KEY_TIMEZONE_OFFSET};
  for (const char* key : keys) {
    EXPECT_LE(strlen(key), 15u) << key;
  }
}

// Bench QC of a bare controller (no ESC, no BMS attached): the operator's
// button confirm turns the two CAN checks into SKIP and QC can pass; without
// the confirm they FAIL.
TEST_F(QcGateEmu, BareControllerQcPassesOnlyWithOperatorSkipConfirm) {
  g_emu.hw.escPresent = false;
  g_emu.hw.bmsPresent = false;
  ASSERT_TRUE(emuBoot().qcRan);
  EXPECT_TRUE(g_emu.screen.bannerPassed) << g_emu.screen.bannerDetail;
  EXPECT_NE(g_emu.screen.bannerDetail.find("esc(skip)"), std::string::npos);

  emuNvsWipe();
  g_emu.op.confirmsSkips = false;
  ASSERT_TRUE(emuBoot().qcRan);
  EXPECT_FALSE(g_emu.screen.bannerPassed);
  EXPECT_EQ(factoryU8("qc_passed"), 0);
}

// ===========================================================================
// Calibration is capture-only in 8.1
// ===========================================================================

// QC stores pot_cal/pot_min/pot_max, but the live mapping stays the fixed
// 0..4095 curve and never touches NVS.
TEST_F(ThrottleCalEmu, QcCalibrationIsCaptureOnlyLiveMappingUnchanged) {
  std::vector<int> refFull, refChill, refSport;
  for (int raw = 0; raw <= 4095; raw++) {
    refFull.push_back(map(raw, 0, 4095, ESC_MIN_PWM, ESC_MAX_PWM));
    refChill.push_back(map(raw, 0, 4095, ESC_MIN_PWM, CHILL_MODE_MAX_PWM));
    refSport.push_back(map(raw, 0, 4095, ESC_MIN_PWM, ESC_MAX_PWM));
  }

  ASSERT_TRUE(emuBoot().qcRan);
  const FactoryCal cal = factoryGetCal();
  ASSERT_TRUE(cal.calibrated);
  EXPECT_EQ(cal.potMin, 150);
  EXPECT_EQ(cal.potMax, 3950);

  emuBootToGate();
  emuNvsResetStats();
  int mismatches = 0;
  for (int raw = 0; raw <= 4095; raw++) {
    const uint16_t r = static_cast<uint16_t>(raw);
    mismatches += potRawToPwm(r) != refFull[raw];
    mismatches += potRawToModePwm(r, 0) != refChill[raw];
    mismatches += potRawToModePwm(r, 1) != refSport[raw];
  }
  g_emu.op.idleRaw = 150;
  for (int i = 0; i < 16; i++) getSmoothedThrottlePwm(1);
  EXPECT_EQ(mismatches, 0);
  EXPECT_EQ(emuNvsStats(kFactoryNs).opens, 0);
  EXPECT_EQ(emuNvsStats(kUserNs).opens, 0);

  // The captured idle (150) still commands above ESC_MIN_PWM and the captured
  // full press (3950) below ESC_MAX_PWM: the calibration is not applied.
  EXPECT_GT(potRawToPwm(150), ESC_MIN_PWM);
  EXPECT_LT(potRawToPwm(3950), ESC_MAX_PWM);
  EXPECT_EQ(getSmoothedThrottlePwm(1), potRawToModePwm(150, 1));
}

// Source check: nothing in src/ outside their own files calls the calibration
// getter or the v8.2 calibrated mapping, so no flight path can read them.
TEST(ThrottleCalEmuSource, CalibrationReadersHaveNoCallersInFirmware) {
  namespace fs = std::filesystem;
  const std::string anchor = readRepoFile("src/sp140/factory_settings.cpp");
  ASSERT_FALSE(anchor.empty());
  std::string srcRoot = "src";
  const std::string here = __FILE__;
  const size_t at = here.find("test/test_emu_settings/");
  if (at != std::string::npos && fs::exists(here.substr(0, at) + "src")) {
    srcRoot = here.substr(0, at) + "src";
  }
  std::vector<std::string> callers;
  for (const auto& entry : fs::recursive_directory_iterator(srcRoot)) {
    if (!entry.is_regular_file()) continue;
    const std::string path = entry.path().generic_string();
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string text = ss.str();
    const bool own = path.find("factory_settings.cpp") != std::string::npos ||
                     path.find("qc_logic.cpp") != std::string::npos;
    if (own) continue;
    if (text.find("factoryGetCal(") != std::string::npos) callers.push_back(path + ": factoryGetCal");
    if (text.find("qcPotRawToPwmCalibrated(") != std::string::npos) {
      callers.push_back(path + ": qcPotRawToPwmCalibrated");
    }
  }
  EXPECT_TRUE(callers.empty()) << ::testing::PrintToString(callers);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
