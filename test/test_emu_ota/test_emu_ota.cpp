// Host emulation of the BLE OTA firmware-update service.
//
// Builds the real src/sp140/ble/ota_service.cpp against the fakes in
// test/emu_stubs/ota (ESP-IDF OTA API, FreeRTOS mutex, NimBLE, Arduino clock)
// and drives it with the same packets the Espressif esp-ble-ota app sends.
// Run with: pio test -e native-emu-ota
#include <gtest/gtest.h>

#include <array>
#include <climits>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Arduino.h"
#ifndef EMU_OTA_ARDUINO_STUB
#error "test/emu_stubs/ota must come before test/native_stubs on the include path"
#endif

#include "../../src/sp140/ble/ota_service.cpp"

// Globals and BLE-core hooks that live in main.cpp / ble_core.cpp on the device.
volatile DeviceState currentState = DISARMED;
void requestFastConnParams() { emu::recordEvent("conn_fast"); }
void requestNormalConnParams() { emu::recordEvent("conn_normal"); }

namespace {

using Bytes = std::vector<uint8_t>;

const unsigned long kBootMs = 1000000;
const size_t kSector = 4096;
const uint16_t kNoResponse = 0xFFFF;
const uint32_t kSlotSize = 0x330000;  // app0/app1 in default_8MB.csv

// Espressif app CRC: CRC-16/XMODEM (poly 0x1021, init 0, no reflection).
uint16_t refCrc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(p[i]) << 8));
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); }
uint8_t hi(uint16_t v) { return static_cast<uint8_t>(v >> 8); }

// Command packet: ID(2) + payload(16) + CRC16(2), little-endian.
Bytes makeCommand(uint16_t id, const std::array<uint8_t, 16>& payload = {}) {
  Bytes p(20, 0);
  p[0] = lo(id);
  p[1] = hi(id);
  std::copy(payload.begin(), payload.end(), p.begin() + 2);
  const uint16_t crc = refCrc16(p.data(), 18);
  p[18] = lo(crc);
  p[19] = hi(crc);
  return p;
}

// START payload bytes 0-3 (packet bytes 2-5) = image length, little-endian.
Bytes startCommand(uint32_t imageLen) {
  std::array<uint8_t, 16> payload = {};
  for (int i = 0; i < 4; ++i) payload[i] = static_cast<uint8_t>(imageLen >> (8 * i));
  return makeCommand(0x0001, payload);
}

Bytes endCommand() { return makeCommand(0x0002); }

// Data packets for one sector: Sector(2) + Seq(1) + data. Seq counts from 0;
// the last packet has Seq 0xFF and carries the sector's CRC16 after its data.
std::vector<Bytes> sectorPackets(uint16_t sector, const Bytes& data, size_t chunk = 200) {
  std::vector<Bytes> out;
  size_t off = 0;
  uint8_t seq = 0;
  while (data.size() - off > chunk) {
    Bytes p = {lo(sector), hi(sector), seq++};
    p.insert(p.end(), data.begin() + off, data.begin() + off + chunk);
    out.push_back(p);
    off += chunk;
  }
  Bytes last = {lo(sector), hi(sector), 0xFF};
  last.insert(last.end(), data.begin() + off, data.end());
  const uint16_t crc = refCrc16(data.data(), data.size());
  last.push_back(lo(crc));
  last.push_back(hi(crc));
  out.push_back(last);
  return out;
}

Bytes makeImage(size_t n, uint32_t seed = 0x5EED) {
  Bytes img(n);
  uint32_t x = seed;
  for (auto& b : img) {
    x = x * 1664525u + 1013904223u;
    b = static_cast<uint8_t>(x >> 24);
  }
  return img;
}

Bytes sectorOf(const Bytes& img, size_t idx) {
  const size_t start = idx * kSector;
  const size_t end = std::min(img.size(), start + kSector);
  return Bytes(img.begin() + start, img.begin() + end);
}

size_t sectorCount(const Bytes& img) { return (img.size() + kSector - 1) / kSector; }

struct CmdResponse {
  uint16_t id;
  uint16_t ackId;
  uint16_t status;
  bool crcOk;
  size_t len;
};

struct FwAck {
  uint16_t sector;
  uint16_t status;
  uint16_t expected;
};

}  // namespace

class OtaEmu : public ::testing::Test {
 protected:
  void SetUp() override {
    emu::reset(kBootMs);
    powerOn();
    server_ = std::make_unique<NimBLEServer>();
    initOtaBleService(server_.get());
    ASSERT_EQ(server_->services.size(), 1u);
    service_ = server_->services[0].get();
    cmd_ = service_->find(OTA_COMMAND_UUID);
    data_ = service_->find(OTA_RECV_FW_UUID);
    ASSERT_NE(cmd_, nullptr);
    ASSERT_NE(data_, nullptr);
    emu::events.clear();
  }

  void TearDown() override {
    EXPECT_EQ(emu::unlockedFlashCalls, 0) << "esp_ota_* called without the OTA state lock";
    EXPECT_EQ(emu::deadlocks, 0) << "blocking take on an already-held OTA state lock";
    EXPECT_FALSE(emu::otaLockHeld()) << "OTA state lock not released";
  }

  // Power-on values of ota_service.cpp's file-scope state. On the device the
  // reboot provides these; the emulation resets them between tests.
  static void powerOn() {
    otaStateMutex = nullptr;
    otaInProgress = false;
    otaRebootAtMs = 0;
    lastOtaActivityMs = 0;
    resetOtaState();
    pRecvFwChar = nullptr;
    pProgressChar = nullptr;
    pCommandChar = nullptr;
    pCustomerChar = nullptr;
    currentState = DISARMED;
  }

  // Mirrors toggleArm() in main.cpp: from DISARMED it holds OtaArmGuard across
  // the OTA check and the ARMED transition, and refuses when the guard can't be
  // held (a START is erasing) or an update is in progress.
  static bool tryArm() {
    OtaArmGuard guard(pdMS_TO_TICKS(100));
    if (!guard.held() || isOtaInProgress()) return false;
    currentState = ARMED;
    return true;
  }
  static bool armingAllowed() {
    OtaArmGuard guard(pdMS_TO_TICKS(100));
    return guard.held() && !isOtaInProgress();
  }

  // Mirrors BleServerConnectionCallbacks::onDisconnect() in ble_core.cpp.
  static void disconnect() {
    emu::recordEvent("disconnect");
    if (isOtaInProgress()) abortOta();
  }

  // bleNotifyTask: checkOtaTimeout() every 20 ms. Stops at ESP.restart().
  static void runNotifyTask(unsigned long ms) {
    for (unsigned long t = 0; t < ms && emu::restartCount == 0; t += 20) {
      emu::advanceMs(20);
      checkOtaTimeout();
    }
  }

  // Writes a command and returns the status of the single response it
  // produced, or kNoResponse.
  uint16_t command(const Bytes& packet) {
    const size_t before = cmd_->notifications.size();
    cmd_->emuWrite(packet);
    if (cmd_->notifications.size() != before + 1) return kNoResponse;
    return lastCmdResponse().status;
  }
  uint16_t start(uint32_t imageLen) { return command(startCommand(imageLen)); }
  uint16_t end() { return command(endCommand()); }

  void sendData(const Bytes& packet) { data_->emuWrite(packet); }

  // Sends one sector and returns the status of the ACK it produced, or
  // kNoResponse if nothing was acknowledged.
  uint16_t sendSector(uint16_t idx, const Bytes& data, size_t chunk = 200) {
    const size_t before = data_->notifications.size();
    for (const auto& p : sectorPackets(idx, data, chunk)) sendData(p);
    if (data_->notifications.size() == before) return kNoResponse;
    return lastFwAck().status;
  }

  ::testing::AssertionResult sendImage(const Bytes& img, size_t chunk = 200) {
    for (size_t s = 0; s < sectorCount(img); ++s) {
      const uint16_t status = sendSector(static_cast<uint16_t>(s), sectorOf(img, s), chunk);
      if (status != 0x0000) {
        return ::testing::AssertionFailure() << "sector " << s << " ACK status " << status;
      }
      if (lastFwAck().sector != s) {
        return ::testing::AssertionFailure() << "sector " << s << " ACKed as " << lastFwAck().sector;
      }
    }
    return ::testing::AssertionSuccess();
  }

  // START, every sector, END. Leaves the device in the reboot window.
  ::testing::AssertionResult completeTransfer(const Bytes& img) {
    const uint16_t s = start(static_cast<uint32_t>(img.size()));
    if (s != 0x0000) return ::testing::AssertionFailure() << "START status " << s;
    ::testing::AssertionResult sent = sendImage(img);
    if (!sent) return sent;
    const uint16_t e = end();
    if (e != 0x0000) return ::testing::AssertionFailure() << "END status " << e;
    return ::testing::AssertionSuccess();
  }

  std::vector<CmdResponse> cmdResponses() const {
    std::vector<CmdResponse> out;
    for (const auto& n : cmd_->notifications) {
      CmdResponse r = {};
      r.len = n.size();
      if (n.size() >= 20) {
        r.id = static_cast<uint16_t>(n[0] | (n[1] << 8));
        r.ackId = static_cast<uint16_t>(n[2] | (n[3] << 8));
        r.status = static_cast<uint16_t>(n[4] | (n[5] << 8));
        r.crcOk = refCrc16(n.data(), 18) == static_cast<uint16_t>(n[18] | (n[19] << 8));
      }
      out.push_back(r);
    }
    return out;
  }
  CmdResponse lastCmdResponse() const {
    const std::vector<CmdResponse> all = cmdResponses();
    return all.empty() ? CmdResponse{kNoResponse, kNoResponse, kNoResponse, false, 0} : all.back();
  }

  std::vector<FwAck> fwAcks() const {
    std::vector<FwAck> out;
    for (const auto& n : data_->notifications) {
      if (n.size() < 6) continue;
      out.push_back({static_cast<uint16_t>(n[0] | (n[1] << 8)),
                     static_cast<uint16_t>(n[2] | (n[3] << 8)),
                     static_cast<uint16_t>(n[4] | (n[5] << 8))});
    }
    return out;
  }
  FwAck lastFwAck() const {
    const std::vector<FwAck> all = fwAcks();
    return all.empty() ? FwAck{kNoResponse, kNoResponse, kNoResponse} : all.back();
  }

  std::unique_ptr<NimBLEServer> server_;
  NimBLEService* service_ = nullptr;
  NimBLECharacteristic* cmd_ = nullptr;
  NimBLECharacteristic* data_ = nullptr;
};

// ---------------------------------------------------------------------------
// GATT surface
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, CommandCharacteristicRequiresEncryptedWrite) {
  EXPECT_EQ(cmd_->getProperties(),
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC |
            NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
}

TEST_F(OtaEmu, DataCharacteristicRequiresEncryptedWrite) {
  EXPECT_EQ(data_->getProperties(),
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE_ENC |
            NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
}

TEST_F(OtaEmu, ProgressAndCustomerCharacteristicsAreNotifyOnly) {
  NimBLECharacteristic* progress = service_->find(OTA_PROGRESS_UUID);
  NimBLECharacteristic* customer = service_->find(OTA_CUSTOMER_UUID);
  ASSERT_NE(progress, nullptr);
  ASSERT_NE(customer, nullptr);
  EXPECT_EQ(progress->getProperties(), NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
  EXPECT_EQ(customer->getProperties(), NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
}

TEST_F(OtaEmu, ServiceUsesEspressifOtaUuidsAndIsStarted) {
  EXPECT_EQ(service_->getUUID().toString(), "00008018-0000-1000-8000-00805f9b34fb");
  EXPECT_EQ(cmd_->getUUID().toString(), "00008022-0000-1000-8000-00805f9b34fb");
  EXPECT_EQ(data_->getUUID().toString(), "00008020-0000-1000-8000-00805f9b34fb");
  EXPECT_EQ(service_->characteristics.size(), 4u);
  EXPECT_TRUE(service_->started);
  EXPECT_NE(cmd_->getCallbacks(), nullptr);
  EXPECT_NE(data_->getCallbacks(), nullptr);
}

// Relies on NimBLE enforcing WRITE_ENC at the ATT layer (modelled in emuWrite).
TEST_F(OtaEmu, UnpairedPhoneCannotStartEraseOrSendData) {
  EXPECT_FALSE(cmd_->emuWrite(startCommand(8192), /*encryptedLink=*/false));
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_TRUE(cmd_->notifications.empty());
  EXPECT_FALSE(isOtaInProgress());

  ASSERT_EQ(start(8192), 0x0000);
  for (const auto& p : sectorPackets(0, sectorOf(makeImage(8192), 0))) {
    EXPECT_FALSE(data_->emuWrite(p, /*encryptedLink=*/false));
  }
  EXPECT_EQ(emu::writeCalls, 0);
  EXPECT_TRUE(data_->notifications.empty());
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, CrcMatchesCrc16XmodemCheckValue) {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(crc16_ccitt(check, sizeof(check)), 0x31C3);
  EXPECT_EQ(refCrc16(check, sizeof(check)), 0x31C3);
}

TEST_F(OtaEmu, CommandResponseIs20ByteFrameWithValidCrc) {
  ASSERT_EQ(start(4096), 0x0000);
  ASSERT_EQ(cmd_->notifications.size(), 1u);
  const Bytes& frame = cmd_->notifications[0];
  ASSERT_EQ(frame.size(), 20u);
  const CmdResponse r = lastCmdResponse();
  EXPECT_EQ(r.id, 0x0003);
  EXPECT_EQ(r.ackId, 0x0001);
  EXPECT_EQ(r.status, 0x0000);
  EXPECT_TRUE(r.crcOk);
  for (size_t i = 6; i < 18; ++i) EXPECT_EQ(frame[i], 0) << "padding byte " << i;
}

// ---------------------------------------------------------------------------
// START
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, StartAcceptedWith0x0000) {
  EXPECT_EQ(start(123456), 0x0000);
  EXPECT_EQ(lastCmdResponse().ackId, 0x0001);
  EXPECT_EQ(emu::beginCalls, 1);
  EXPECT_EQ(emu::beginImageSize, 123456u);
  EXPECT_EQ(emu::liveHandles.size(), 1u);
  EXPECT_TRUE(isOtaInProgress());
}

// The request must not be in flight while esp_ota_begin() erases: the phone
// applies the change mid-erase and the link is lost (bench, Pixel 9 Pro).
TEST_F(OtaEmu, ConnParamsRequestedAfterEraseBeforeAccept) {
  ASSERT_EQ(start(8192), 0x0000);
  const int fast = emu::indexOf("conn_fast");
  const int begin = emu::indexOf("ota_begin");
  const int response = emu::indexOf("notify 8022");
  ASSERT_GE(fast, 0);
  ASSERT_GE(begin, 0);
  EXPECT_LT(begin, fast);
  EXPECT_LT(fast, response);
  EXPECT_EQ(emu::countOf("conn_fast"), 1);
}

TEST_F(OtaEmu, StartWhileArmedRefusedWith0x0002) {
  currentState = ARMED;
  EXPECT_EQ(start(8192), 0x0002);
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_EQ(emu::indexOf("ota_get_next_update_partition"), -1);
  EXPECT_EQ(emu::indexOf("conn_fast"), -1);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, StartWhileCruisingRefusedWith0x0002) {
  currentState = ARMED_CRUISING;
  EXPECT_EQ(start(8192), 0x0002);
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, StartImageTooLargeRefusedWith0x0003) {
  EXPECT_EQ(start(kSlotSize + 1), 0x0003);
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_EQ(emu::indexOf("conn_fast"), -1);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, StartImageExactlySlotSizeAccepted) {
  EXPECT_EQ(start(kSlotSize), 0x0000);
  EXPECT_EQ(emu::beginImageSize, kSlotSize);
}

TEST_F(OtaEmu, StartWithoutUpdatePartitionRefusedWith0x0004) {
  emu::faults.noUpdatePartition = true;
  EXPECT_EQ(start(8192), 0x0004);
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, StartEraseFailureRefusedWith0x0004) {
  emu::faults.beginErr = ESP_ERR_FLASH_BASE + 1;
  EXPECT_EQ(start(8192), 0x0004);
  EXPECT_EQ(emu::beginCalls, 1);
  EXPECT_EQ(emu::abortCalls, 0);  // no handle was issued
  EXPECT_TRUE(emu::liveHandles.empty());
  EXPECT_GT(emu::indexOf("conn_normal"), emu::indexOf("conn_fast"));
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, StartWithBadCrcRefusedWith0x0001) {
  Bytes packet = startCommand(8192);
  packet[18] ^= 0x01;
  EXPECT_EQ(command(packet), 0x0001);
  EXPECT_EQ(lastCmdResponse().ackId, 0x0001);
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, CommandShorterThan20BytesIgnored) {
  Bytes packet = startCommand(8192);
  packet.pop_back();
  EXPECT_EQ(command(packet), kNoResponse);
  EXPECT_EQ(emu::beginCalls, 0);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, UnknownCommandIdIgnored) {
  EXPECT_EQ(command(makeCommand(0x0007)), kNoResponse);
  EXPECT_FALSE(isOtaInProgress());
  EXPECT_TRUE(emu::events.empty());
}

TEST_F(OtaEmu, RepeatedStartAbortsPreviousSessionFirst) {
  const Bytes img = makeImage(3 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);

  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_EQ(emu::abortNotFound, 0);  // the live handle was aborted, not leaked
  EXPECT_EQ(emu::beginCalls, 2);
  EXPECT_GE(emu::indexOf("ota_abort"), 0);
  EXPECT_LT(emu::indexOf("ota_abort"), emu::lastIndexOf("ota_begin"));
  EXPECT_EQ(emu::liveHandles.size(), 1u);

  ASSERT_TRUE(sendImage(img));  // sector numbering restarts at 0
  EXPECT_EQ(end(), 0x0000);
  EXPECT_EQ(emu::flashImage, img);
}

// ---------------------------------------------------------------------------
// Arming interlock
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, ArmingBlockedFromStartThroughErase) {
  EXPECT_TRUE(armingAllowed());
  bool allowedDuringErase = true;
  emu::faults.eraseMs = 4000;
  emu::onBegin = [&] { allowedDuringErase = armingAllowed(); };
  ASSERT_EQ(start(8192), 0x0000);
  EXPECT_FALSE(allowedDuringErase);
  EXPECT_FALSE(armingAllowed());
}

TEST_F(OtaEmu, ArmingBlockedUntilReboot) {
  const Bytes img = makeImage(2 * kSector + 100);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  EXPECT_FALSE(armingAllowed());
  ASSERT_EQ(sendSector(1, sectorOf(img, 1)), 0x0000);
  ASSERT_EQ(sendSector(2, sectorOf(img, 2)), 0x0000);
  ASSERT_EQ(end(), 0x0000);
  EXPECT_FALSE(armingAllowed());
  disconnect();
  EXPECT_FALSE(armingAllowed());
  emu::advanceMs(OTA_REBOOT_DELAY_MS - 20);
  checkOtaTimeout();
  EXPECT_FALSE(armingAllowed());
  EXPECT_EQ(emu::restartCount, 0);
  runNotifyTask(100);
  EXPECT_EQ(emu::restartCount, 1);
}

TEST_F(OtaEmu, ArmingUnblockedAfterDisconnectAbort) {
  ASSERT_EQ(start(8192), 0x0000);
  EXPECT_FALSE(armingAllowed());
  disconnect();
  EXPECT_TRUE(armingAllowed());
}

TEST_F(OtaEmu, ArmingUnblockedAfterIdleTimeoutAbort) {
  ASSERT_EQ(start(8192), 0x0000);
  runNotifyTask(OTA_TIMEOUT_MS + 100);
  EXPECT_TRUE(armingAllowed());
}

TEST_F(OtaEmu, ArmingUnblockedAfterRefusedStart) {
  EXPECT_EQ(start(kSlotSize + 1), 0x0003);
  EXPECT_TRUE(armingAllowed());
  emu::faults.beginErr = ESP_FAIL;
  EXPECT_EQ(start(8192), 0x0004);
  EXPECT_TRUE(armingAllowed());
}

TEST_F(OtaEmu, ArmingUnblockedAfterFailedEnd) {
  ASSERT_EQ(start(8192), 0x0000);
  EXPECT_EQ(end(), 0x0001);  // nothing sent: size mismatch
  EXPECT_TRUE(armingAllowed());
}

// ---------------------------------------------------------------------------
// Happy path and deferred reboot
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, HappyPathWritesImageInOrderAndDefersReboot) {
  const Bytes img = makeImage(3 * kSector + 1000);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_TRUE(sendImage(img));
  EXPECT_EQ(emu::writeCalls, 4);
  EXPECT_EQ(emu::flashImage, img);
  const std::vector<FwAck> acks = fwAcks();
  ASSERT_EQ(acks.size(), 4u);
  for (uint16_t i = 0; i < 4; ++i) {
    EXPECT_EQ(acks[i].sector, i);
    EXPECT_EQ(acks[i].status, 0x0000);
  }

  EXPECT_EQ(end(), 0x0000);
  EXPECT_EQ(lastCmdResponse().ackId, 0x0002);
  EXPECT_EQ(emu::endCalls, 1);
  EXPECT_EQ(emu::setBootCalls, 1);
  EXPECT_EQ(emu::bootPartition, &emu::updatePartition);
  EXPECT_LT(emu::indexOf("ota_end"), emu::indexOf("set_boot_partition"));
  EXPECT_EQ(emu::restartCount, 0) << "ESP.restart() ran inside the END write callback";
  EXPECT_EQ(emu::abortCalls, 0);
}

TEST_F(OtaEmu, RebootWaitsForOtaRebootDelay) {
  EXPECT_EQ(OTA_REBOOT_DELAY_MS, 1000u);
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 0);
  emu::advanceMs(OTA_REBOOT_DELAY_MS - 1);
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 0);
  emu::advanceMs(1);
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 1);
}

TEST_F(OtaEmu, RebootHappensExactlyOnceOn50HzNotifyTicks) {
  ASSERT_TRUE(completeTransfer(makeImage(2 * kSector)));
  const unsigned long endAt = emu::nowMs;
  runNotifyTask(5000);
  EXPECT_EQ(emu::restartCount, 1);
  EXPECT_GE(emu::restartAtMs - endAt, OTA_REBOOT_DELAY_MS);
  EXPECT_LT(emu::restartAtMs - endAt, OTA_REBOOT_DELAY_MS + 20);
}

TEST_F(OtaEmu, EndAckSentBeforeReboot) {
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  runNotifyTask(2000);
  ASSERT_EQ(emu::restartCount, 1);
  const int lastCmdNotify = emu::lastIndexOf("notify 8022");
  EXPECT_GT(lastCmdNotify, emu::indexOf("set_boot_partition"));
  EXPECT_LT(lastCmdNotify, emu::indexOf("restart"));
  EXPECT_EQ(lastCmdResponse().ackId, 0x0002);
  EXPECT_EQ(lastCmdResponse().status, 0x0000);
}

// ---------------------------------------------------------------------------
// END failures
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, EndSizeMismatchRefusedWith0x0001AndAborts) {
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  EXPECT_EQ(end(), 0x0001);
  EXPECT_EQ(emu::endCalls, 0);
  EXPECT_EQ(emu::setBootCalls, 0);
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_EQ(emu::abortNotFound, 0);
  EXPECT_GE(emu::indexOf("conn_normal"), 0);
  EXPECT_FALSE(isOtaInProgress());
  runNotifyTask(5000);
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, ImageLongerThanDeclaredRefusedAtEnd) {
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(kSector), 0x0000);
  ASSERT_TRUE(sendImage(img));  // sectors past the declared size are not refused
  EXPECT_EQ(end(), 0x0001);
  EXPECT_EQ(emu::setBootCalls, 0);
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, EndWhenImageValidationFailsRefusedAndAborts) {
  emu::faults.endErr = ESP_ERR_OTA_VALIDATE_FAILED;
  const Bytes img = makeImage(kSector + 10);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_TRUE(sendImage(img));
  EXPECT_NE(end(), 0x0000);
  EXPECT_EQ(lastCmdResponse().status, 0x0001);
  EXPECT_EQ(emu::setBootCalls, 0);
  EXPECT_EQ(emu::bootPartition, nullptr);
  // esp_ota_end() already released the handle; IDF answers the follow-up
  // esp_ota_abort() with ESP_ERR_NOT_FOUND, which is harmless.
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_EQ(emu::abortNotFound, 1);
  EXPECT_FALSE(isOtaInProgress());
  runNotifyTask(5000);
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, EndWhenSetBootPartitionFailsRefusedAndAborts) {
  emu::faults.setBootErr = ESP_ERR_OTA_VALIDATE_FAILED;
  const Bytes img = makeImage(kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_TRUE(sendImage(img));
  EXPECT_EQ(end(), 0x0001);
  EXPECT_EQ(emu::setBootCalls, 1);
  EXPECT_EQ(emu::bootPartition, nullptr);
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_FALSE(isOtaInProgress());
  runNotifyTask(5000);
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, EndWithoutSessionRefusedWith0x0001) {
  EXPECT_EQ(end(), 0x0001);
  EXPECT_EQ(emu::endCalls, 0);
  EXPECT_EQ(emu::setBootCalls, 0);
}

TEST_F(OtaEmu, ZeroLengthImageNeverBecomesBootPartition) {
  EXPECT_EQ(start(0), 0x0000);  // accepted; IDF's esp_ota_end() refuses it
  EXPECT_EQ(end(), 0x0001);
  EXPECT_EQ(emu::setBootCalls, 0);
  EXPECT_FALSE(isOtaInProgress());
}

// ---------------------------------------------------------------------------
// Disconnects
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, DisconnectMidTransferAbortsAndResetsState) {
  const Bytes img = makeImage(3 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  disconnect();
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_EQ(emu::abortNotFound, 0);
  EXPECT_TRUE(emu::liveHandles.empty());
  EXPECT_GT(emu::indexOf("conn_normal"), emu::indexOf("ota_abort"));
  EXPECT_FALSE(isOtaInProgress());
  EXPECT_EQ(receivedBytes, 0u);
  EXPECT_EQ(currentSectorIndex, 0u);

  const size_t acks = data_->notifications.size();
  sendSector(1, sectorOf(img, 1));
  EXPECT_EQ(emu::writeCalls, 1);
  EXPECT_EQ(data_->notifications.size(), acks);
  EXPECT_EQ(end(), 0x0001);
}

TEST_F(OtaEmu, RetryAfterDisconnectSucceeds) {
  const Bytes img = makeImage(3 * kSector + 77);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  ASSERT_EQ(sendSector(1, sectorOf(img, 1)), 0x0000);
  disconnect();
  ASSERT_FALSE(isOtaInProgress());

  ASSERT_TRUE(completeTransfer(img));
  EXPECT_EQ(emu::beginCalls, 2);
  EXPECT_EQ(emu::flashImage, img);
  EXPECT_EQ(emu::setBootCalls, 1);
  runNotifyTask(2000);
  EXPECT_EQ(emu::restartCount, 1);
}

TEST_F(OtaEmu, DisconnectDuringRebootWindowDoesNotAbort) {
  ASSERT_TRUE(completeTransfer(makeImage(2 * kSector)));
  const unsigned long endAt = emu::nowMs;
  emu::advanceMs(300);
  disconnect();
  EXPECT_EQ(emu::abortCalls, 0);
  EXPECT_EQ(emu::indexOf("conn_normal"), -1);
  EXPECT_TRUE(isOtaInProgress());
  EXPECT_EQ(emu::bootPartition, &emu::updatePartition);
  runNotifyTask(2000);
  EXPECT_EQ(emu::restartCount, 1);
  EXPECT_GE(emu::restartAtMs - endAt, OTA_REBOOT_DELAY_MS);
  EXPECT_LT(emu::restartAtMs - endAt, OTA_REBOOT_DELAY_MS + 20);
}

TEST_F(OtaEmu, AbortOtaDuringRebootWindowIsIgnored) {
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  abortOta();
  EXPECT_EQ(emu::abortCalls, 0);
  EXPECT_TRUE(isOtaInProgress());
  runNotifyTask(2000);
  EXPECT_EQ(emu::restartCount, 1);
}

// ---------------------------------------------------------------------------
// Commands while the reboot is pending
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, EndWhileRebootPendingReturns0x0000) {
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  EXPECT_EQ(end(), 0x0000);
  EXPECT_EQ(lastCmdResponse().ackId, 0x0002);
  EXPECT_EQ(emu::endCalls, 1);
  EXPECT_EQ(emu::setBootCalls, 1);
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, StartWhileRebootPendingReturnsGenericError) {
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  EXPECT_EQ(start(8192), 0x0001);
  EXPECT_EQ(lastCmdResponse().ackId, 0x0001);
  EXPECT_EQ(command(makeCommand(0x0007)), 0x0001);
  EXPECT_EQ(emu::beginCalls, 1);
  EXPECT_EQ(emu::abortCalls, 0);
  EXPECT_TRUE(isOtaInProgress());
  runNotifyTask(2000);
  EXPECT_EQ(emu::restartCount, 1);
}

TEST_F(OtaEmu, DataWhileRebootPendingIgnored) {
  const Bytes img = makeImage(kSector);
  ASSERT_TRUE(completeTransfer(img));
  const size_t acks = data_->notifications.size();
  EXPECT_EQ(sendSector(1, img), kNoResponse);
  EXPECT_EQ(sendSector(0, img), kNoResponse);
  EXPECT_EQ(emu::writeCalls, 1);
  EXPECT_EQ(data_->notifications.size(), acks);
}

// ---------------------------------------------------------------------------
// Idle timeout
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, IdleTimeoutAbortsAfterOtaTimeoutMsNotBefore) {
  EXPECT_EQ(OTA_TIMEOUT_MS, 30000u);
  ASSERT_EQ(start(8192), 0x0000);
  emu::advanceMs(OTA_TIMEOUT_MS);
  checkOtaTimeout();
  EXPECT_TRUE(isOtaInProgress());
  EXPECT_EQ(emu::abortCalls, 0);
  emu::advanceMs(1);
  checkOtaTimeout();
  EXPECT_FALSE(isOtaInProgress());
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_GT(emu::indexOf("conn_normal"), emu::indexOf("ota_abort"));
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, DataActivityResetsIdleTimer) {
  const Bytes img = makeImage(kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  const std::vector<Bytes> packets = sectorPackets(0, img);
  ASSERT_GE(packets.size(), 6u);
  for (size_t i = 0; i < 5; ++i) {
    runNotifyTask(20000);
    sendData(packets[i]);
    ASSERT_TRUE(isOtaInProgress()) << "aborted after " << (i + 1) * 20 << " s with traffic";
  }
  runNotifyTask(OTA_TIMEOUT_MS - 40);
  EXPECT_TRUE(isOtaInProgress());
  runNotifyTask(100);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, IdleTimerRestartsWhenEraseFinishes) {
  emu::faults.eraseMs = 25000;
  ASSERT_EQ(start(kSlotSize), 0x0000);
  emu::advanceMs(OTA_TIMEOUT_MS - 1000);
  checkOtaTimeout();
  EXPECT_TRUE(isOtaInProgress()) << "erase time counted against the idle timeout";
  emu::advanceMs(1001);
  checkOtaTimeout();
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, TimeoutCheckDuringEraseSkipsInsteadOfBlocking) {
  emu::faults.eraseMs = OTA_TIMEOUT_MS + 5000;
  emu::onBegin = [] { checkOtaTimeout(); };  // bleNotifyTask tick mid-erase
  ASSERT_EQ(start(kSlotSize), 0x0000);
  ASSERT_FALSE(emu::mutexes.empty());
  EXPECT_GE(emu::mutexes.back()->busyTryTakes, 1);
  EXPECT_EQ(emu::abortCalls, 0);
  EXPECT_TRUE(isOtaInProgress());
}

TEST_F(OtaEmu, TimeoutCheckDuringFlashWriteCannotAbortMidWrite) {
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  emu::onWrite = [] {
    emu::advanceMs(OTA_TIMEOUT_MS + 1000);  // a write that stalls past the timeout
    checkOtaTimeout();
  };
  EXPECT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  EXPECT_EQ(emu::abortCalls, 0) << "esp_ota_abort() ran while esp_ota_write() was in progress";
  EXPECT_EQ(emu::flashImage.size(), kSector);
  EXPECT_GE(emu::mutexes.back()->busyTryTakes, 1);
}

TEST_F(OtaEmu, ShortDataPacketDoesNotCountAsActivity) {
  ASSERT_EQ(start(8192), 0x0000);
  emu::advanceMs(20000);
  sendData({0x00, 0x00});
  EXPECT_TRUE(data_->notifications.empty());
  emu::advanceMs(OTA_TIMEOUT_MS - 20000 + 1);
  checkOtaTimeout();
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, TimeoutCheckWithoutSessionDoesNothing) {
  runNotifyTask(2 * OTA_TIMEOUT_MS);
  EXPECT_TRUE(emu::events.empty());
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, IdleTimeoutCorrectAcrossMillisWraparound) {
  emu::nowMs = ULONG_MAX - 10000;
  ASSERT_EQ(start(8192), 0x0000);
  emu::advanceMs(20000);  // clock wraps
  checkOtaTimeout();
  EXPECT_TRUE(isOtaInProgress());
  emu::advanceMs(OTA_TIMEOUT_MS - 20000 + 1);
  checkOtaTimeout();
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaEmu, RebootScheduleCorrectAcrossMillisWraparound) {
  emu::nowMs = ULONG_MAX - 500;
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  emu::advanceMs(OTA_REBOOT_DELAY_MS - 1);  // clock wraps
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 0);
  emu::advanceMs(1);
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 1);
}

// timeMillis() + delay == 0 collides with the "no reboot pending" sentinel; the
// firmware bumps it to 1, so the reboot comes 1 ms late but still comes.
TEST_F(OtaEmu, RebootDeadlineWrappingToZeroStillFires) {
  emu::nowMs = ULONG_MAX - OTA_REBOOT_DELAY_MS + 1;
  ASSERT_TRUE(completeTransfer(makeImage(kSector)));
  EXPECT_EQ(static_cast<unsigned long>(otaRebootAtMs), 1u);
  EXPECT_TRUE(isOtaInProgress());
  emu::advanceMs(OTA_REBOOT_DELAY_MS);
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 0);
  emu::advanceMs(1);
  checkOtaTimeout();
  EXPECT_EQ(emu::restartCount, 1);
}

// ---------------------------------------------------------------------------
// Data path
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, DataBeforeStartIgnored) {
  const Bytes img = makeImage(kSector);
  EXPECT_EQ(sendSector(0, img), kNoResponse);
  EXPECT_EQ(emu::writeCalls, 0);
}

TEST_F(OtaEmu, WrongSectorNackedWithExpectedSectorAndRecovers) {
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  const std::vector<Bytes> s0 = sectorPackets(0, sectorOf(img, 0));
  for (size_t i = 0; i < 5; ++i) sendData(s0[i]);  // half a sector buffered
  sendData(sectorPackets(5, sectorOf(img, 1))[0]);
  ASSERT_EQ(fwAcks().size(), 1u);
  EXPECT_EQ(lastFwAck().sector, 5);
  EXPECT_EQ(lastFwAck().status, 0x0002);
  EXPECT_EQ(lastFwAck().expected, 0);
  EXPECT_EQ(emu::writeCalls, 0);

  ASSERT_TRUE(sendImage(img));  // partial sector 0 was discarded, not merged
  EXPECT_EQ(end(), 0x0000);
  EXPECT_EQ(emu::flashImage, img);
}

TEST_F(OtaEmu, SectorCrcMismatchNackedAndRetryable) {
  const Bytes img = makeImage(kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  std::vector<Bytes> packets = sectorPackets(0, img);
  packets.back().back() ^= 0xFF;
  for (const auto& p : packets) sendData(p);
  EXPECT_EQ(lastFwAck().sector, 0);
  EXPECT_EQ(lastFwAck().status, 0x0001);
  EXPECT_EQ(emu::writeCalls, 0);
  EXPECT_TRUE(isOtaInProgress());

  EXPECT_EQ(sendSector(0, img), 0x0000);
  EXPECT_EQ(emu::writeCalls, 1);
  EXPECT_EQ(end(), 0x0000);
  EXPECT_EQ(emu::flashImage, img);
}

// Seq numbers are not checked; reordering is caught by the sector CRC.
TEST_F(OtaEmu, OutOfOrderPacketsCaughtBySectorCrc) {
  const Bytes img = makeImage(kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  std::vector<Bytes> packets = sectorPackets(0, img);
  std::swap(packets[1], packets[2]);
  for (const auto& p : packets) sendData(p);
  EXPECT_EQ(lastFwAck().status, 0x0001);
  EXPECT_EQ(emu::writeCalls, 0);
  EXPECT_EQ(sendSector(0, img), 0x0000);
}

TEST_F(OtaEmu, DuplicatedPacketCaughtBySectorCrc) {
  const Bytes img = makeImage(kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  std::vector<Bytes> packets = sectorPackets(0, img);
  packets.insert(packets.begin() + 3, packets[3]);
  for (const auto& p : packets) sendData(p);
  EXPECT_EQ(lastFwAck().status, 0x0001);
  EXPECT_EQ(emu::writeCalls, 0);
}

TEST_F(OtaEmu, SectorOverflowNackedWithLengthError) {
  ASSERT_EQ(start(2 * kSector), 0x0000);
  const Bytes chunk = makeImage(500);
  for (int i = 0; i < 9; ++i) {  // 9 x 500 = 4500 > 4096
    Bytes p = {0x00, 0x00, static_cast<uint8_t>(i)};
    p.insert(p.end(), chunk.begin(), chunk.end());
    sendData(p);
  }
  ASSERT_EQ(fwAcks().size(), 1u);
  EXPECT_EQ(lastFwAck().status, 0x0003);
  EXPECT_EQ(sectorBufferLen, 0u);
  EXPECT_EQ(emu::writeCalls, 0);
  EXPECT_TRUE(isOtaInProgress());
}

TEST_F(OtaEmu, LastPacketOverflowNackedWithLengthError) {
  ASSERT_EQ(start(2 * kSector), 0x0000);
  const Bytes chunk = makeImage(500);
  for (int i = 0; i < 8; ++i) {  // 4000 bytes buffered
    Bytes p = {0x00, 0x00, static_cast<uint8_t>(i)};
    p.insert(p.end(), chunk.begin(), chunk.end());
    sendData(p);
  }
  Bytes last = {0x00, 0x00, 0xFF};
  last.insert(last.end(), chunk.begin(), chunk.begin() + 200);  // 4200 > 4096
  last.push_back(0x00);
  last.push_back(0x00);
  sendData(last);
  ASSERT_EQ(fwAcks().size(), 1u);
  EXPECT_EQ(lastFwAck().status, 0x0003);
  EXPECT_EQ(emu::writeCalls, 0);
}

TEST_F(OtaEmu, FullSize4096ByteSectorAccepted) {
  const Bytes img = makeImage(kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  EXPECT_EQ(sendSector(0, img, 512), 0x0000);  // 7 x 512 + 512 in the last packet
  EXPECT_EQ(emu::flashImage, img);
}

TEST_F(OtaEmu, RetransmittedSectorNotWrittenTwice) {
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  EXPECT_EQ(sendSector(0, sectorOf(img, 0)), 0x0002);
  EXPECT_EQ(lastFwAck().expected, 1);
  EXPECT_EQ(emu::writeCalls, 1);
  ASSERT_EQ(sendSector(1, sectorOf(img, 1)), 0x0000);
  EXPECT_EQ(end(), 0x0000);
  EXPECT_EQ(emu::flashImage, img);
}

TEST_F(OtaEmu, FlashWriteFailureAbortsSession) {
  emu::faults.failWriteOnCall = 2;
  const Bytes img = makeImage(3 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  EXPECT_EQ(sendSector(1, sectorOf(img, 1)), 0x0002);
  EXPECT_EQ(lastFwAck().sector, 1);
  EXPECT_EQ(lastFwAck().expected, 1);
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_FALSE(isOtaInProgress());

  // The app resends sector 1; the aborted session drops it without an ACK.
  EXPECT_EQ(sendSector(1, sectorOf(img, 1)), kNoResponse);
  EXPECT_EQ(emu::writeCalls, 2);
  EXPECT_EQ(end(), 0x0001);
  EXPECT_EQ(emu::setBootCalls, 0);
}

TEST_F(OtaEmu, LastPacketWithoutCrcIgnored) {
  ASSERT_EQ(start(8192), 0x0000);
  sendData({0x00, 0x00, 0xFF, 0xAB});
  EXPECT_TRUE(data_->notifications.empty());
  EXPECT_EQ(emu::writeCalls, 0);
  EXPECT_TRUE(isOtaInProgress());
}

// ---------------------------------------------------------------------------
// Locking
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, EveryFlashCallHoldsOtaStateLock) {
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  ASSERT_EQ(sendSector(0, sectorOf(img, 0)), 0x0000);
  disconnect();
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  runNotifyTask(OTA_TIMEOUT_MS + 100);
  ASSERT_TRUE(completeTransfer(img));
  runNotifyTask(2000);
  ASSERT_EQ(emu::restartCount, 1);
  EXPECT_EQ(emu::unlockedFlashCalls, 0);
  EXPECT_GT(emu::mutexes.back()->takes, 0);
  EXPECT_EQ(emu::mutexes.back()->takes, emu::mutexes.back()->gives);
}

// ---------------------------------------------------------------------------
// Rollback
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, VerifyRollbackLaterReturnsTrue) {
  EXPECT_TRUE(verifyRollbackLater());
}

// verifyRollbackLater() only matters when the bootloader can roll back.
TEST_F(OtaEmu, RollbackEnabledInSdkconfig) {
  const std::string here = __FILE__;
  const size_t pos = here.find("test/test_emu_ota/");
  const std::string root = pos == std::string::npos ? "" : here.substr(0, pos);
  for (const char* name : {"sdkconfig.defaults", "sdkconfig.OpenPPG-CESP32S3-CAN-SP140"}) {
    std::ifstream f(name);
    if (!f) f.open(root + name);
    ASSERT_TRUE(f.good()) << "cannot open " << name;
    std::stringstream ss;
    ss << f.rdbuf();
    EXPECT_NE(ss.str().find("CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y"), std::string::npos) << name;
  }
}

// ---------------------------------------------------------------------------
// Arming vs CMD_START. toggleArm() (button task) needs up to ~50 ms between its
// OTA check and setting ARMED (barometer datum). Before OtaArmGuard, a START on
// the NimBLE host task in that window saw DISARMED, started the update, and
// the controller later rebooted while ARMED. The guard holds the OTA state
// lock across the check and the transition, so the two can only run in order.
// ---------------------------------------------------------------------------

TEST_F(OtaEmu, ArmingDuringStartEraseIsRefusedAndRebootStaysDisarmed) {
  emu::faults.eraseMs = 3000;
  bool armedDuringErase = true;
  emu::onBegin = [&] { armedDuringErase = tryArm(); };  // button pressed mid-erase
  const Bytes img = makeImage(2 * kSector);
  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), 0x0000);
  EXPECT_FALSE(armedDuringErase);
  EXPECT_TRUE(sendImage(img));
  EXPECT_EQ(end(), 0x0000);
  runNotifyTask(5000);
  EXPECT_EQ(emu::restartCount, 1);
  EXPECT_EQ(static_cast<DeviceState>(currentState), DISARMED);
}

TEST_F(OtaEmu, StartWaitsForArmingInProgressThenSeesArmed) {
  {
    // toggleArm() between its OTA check and ARMED: START's onWrite would block
    // on the lock here, so it can only run after the transition.
    OtaArmGuard guard(pdMS_TO_TICKS(100));
    ASSERT_TRUE(guard.held());
    EXPECT_TRUE(emu::otaLockHeld());
    currentState = ARMED;
  }
  EXPECT_FALSE(emu::otaLockHeld());
  EXPECT_EQ(start(2 * kSector), 0x0002);
  EXPECT_EQ(emu::beginCalls, 0);
  runNotifyTask(5000);
  EXPECT_EQ(emu::restartCount, 0);
}

TEST_F(OtaEmu, ArmGuardTimesOutInsteadOfBlockingWhileStartHoldsLock) {
  emu::onBegin = [] {
    OtaArmGuard guard(pdMS_TO_TICKS(100));
    EXPECT_FALSE(guard.held());
  };
  ASSERT_EQ(start(2 * kSector), 0x0000);
  EXPECT_EQ(emu::mutexes.back()->timedOutTakes, 1);
  EXPECT_EQ(emu::deadlocks, 0);
}

// ---------------------------------------------------------------------------
// Sector re-send characterisation (input to the phone apps' retry policy).
//
// A phone that gets no sector ACK re-sends the whole sector. These tests pin
// down what the controller answers to each re-send, mid-transfer at sector 3
// of a 6-sector image, with 9 packets per sector (8 x 500 bytes + 96) as seen
// on the bench. They do not change firmware behaviour.
//
// The fake notify() records every call. On the device NimBLE-Arduino's
// notify() only marks the characteristic and the host sends its *current*
// value later, so ACKs set while a burst of packets is being processed can
// collapse into one notification carrying the last of them. The sequences
// below are therefore what the controller tries to send; the phone sees a
// subset that always includes the last ACK of each burst.
// ---------------------------------------------------------------------------

namespace {

struct PacketAck {
  size_t packet;  // index of the packet (within the call) that produced it
  FwAck ack;
};

bool operator==(const FwAck& a, const FwAck& b) {
  return a.sector == b.sector && a.status == b.status && a.expected == b.expected;
}
bool operator==(const PacketAck& a, const PacketAck& b) {
  return a.packet == b.packet && a.ack == b.ack;
}
void PrintTo(const FwAck& a, std::ostream* os) {
  *os << "[sector " << a.sector << " status " << a.status << " expected " << a.expected << "]";
}
void PrintTo(const PacketAck& p, std::ostream* os) {
  *os << "pkt " << p.packet << " ";
  PrintTo(p.ack, os);
}

const uint16_t kAckOk = 0x0000;
const uint16_t kAckCrc = 0x0001;
const uint16_t kAckSector = 0x0002;
const uint16_t kAckLen = 0x0003;

// Sector index no image can reach (0xFFFF * 4096 > any app slot).
const uint16_t kProbeSector = 0xFFFF;

}  // namespace

class OtaResendEmu : public OtaEmu {
 protected:
  static constexpr uint16_t kN = 3;
  static constexpr size_t kChunk = 500;  // 9 packets per 4096-byte sector

  static std::vector<Bytes> packetsOf(const Bytes& img, size_t s, size_t chunk = kChunk) {
    return sectorPackets(static_cast<uint16_t>(s), sectorOf(img, s), chunk);
  }

  static std::vector<Bytes> without(std::vector<Bytes> packets, size_t idx) {
    packets.erase(packets.begin() + static_cast<long>(idx));
    return packets;
  }

  static std::vector<PacketAck> repeat(size_t count, FwAck ack) {
    std::vector<PacketAck> out;
    for (size_t i = 0; i < count; ++i) out.push_back({i, ack});
    return out;
  }

  // Sends `packets` in order and returns every ACK, tagged with the index of
  // the packet whose write produced it.
  std::vector<PacketAck> play(const std::vector<Bytes>& packets) {
    std::vector<PacketAck> out;
    for (size_t i = 0; i < packets.size(); ++i) {
      const size_t before = data_->notifications.size();
      sendData(packets[i]);
      for (size_t k = before; k < data_->notifications.size(); ++k) {
        const Bytes& n = data_->notifications[k];
        EXPECT_EQ(n.size(), 6u) << "sector ACKs are 6 bytes";
        if (n.size() < 6) continue;
        out.push_back({i, {static_cast<uint16_t>(n[0] | (n[1] << 8)),
                           static_cast<uint16_t>(n[2] | (n[3] << 8)),
                           static_cast<uint16_t>(n[4] | (n[5] << 8))}});
      }
    }
    return out;
  }

  // 3-byte data write with an unreachable sector index.
  std::vector<PacketAck> probe() { return play({{lo(kProbeSector), hi(kProbeSector), 0x00}}); }

  // START, then sectors 0..n-1, each answered with a success ACK.
  ::testing::AssertionResult startAndSendUpTo(const Bytes& img, size_t n, size_t chunk = kChunk) {
    const uint16_t s = start(static_cast<uint32_t>(img.size()));
    if (s != kAckOk) return ::testing::AssertionFailure() << "START status " << s;
    return sendSectors(img, 0, n, chunk);
  }

  ::testing::AssertionResult sendSectors(const Bytes& img, size_t from, size_t to,
                                         size_t chunk = kChunk) {
    for (size_t i = from; i < to; ++i) {
      const std::vector<PacketAck> acks = play(packetsOf(img, i, chunk));
      const FwAck want = {static_cast<uint16_t>(i), kAckOk, 0};
      if (acks.size() != 1 || !(acks[0].ack == want)) {
        return ::testing::AssertionFailure() << "sector " << i << ": " << ::testing::PrintToString(acks);
      }
    }
    return ::testing::AssertionSuccess();
  }

  // Sends sectors from..end, then END; checks the image reached flash once,
  // in order, with the declared total. priorWrites: esp_ota_write calls made
  // in earlier (aborted) sessions.
  ::testing::AssertionResult finishFrom(const Bytes& img, size_t from, size_t chunk = kChunk,
                                        int priorWrites = 0) {
    ::testing::AssertionResult sent = sendSectors(img, from, sectorCount(img), chunk);
    if (!sent) return sent;
    if (receivedBytes != img.size()) {
      return ::testing::AssertionFailure() << "receivedBytes " << receivedBytes << " != " << img.size();
    }
    const uint16_t e = end();
    if (e != kAckOk) return ::testing::AssertionFailure() << "END status " << e;
    if (emu::flashImage != img) return ::testing::AssertionFailure() << "flash image differs";
    if (emu::writeCalls - priorWrites != static_cast<int>(sectorCount(img))) {
      return ::testing::AssertionFailure() << emu::writeCalls - priorWrites
                                           << " esp_ota_write calls for " << sectorCount(img)
                                           << " sectors";
    }
    return ::testing::AssertionSuccess();
  }

  // Every success ACK corresponds to exactly one esp_ota_write, and they name
  // sectors 0, 1, 2, ... in order: no success ACK for data that was not written.
  ::testing::AssertionResult successAcksMatchWrites() const {
    uint16_t next = 0;
    for (const FwAck& a : fwAcks()) {
      if (a.status != kAckOk) continue;
      if (a.sector != next) {
        return ::testing::AssertionFailure() << "success ACK for sector " << a.sector
                                             << ", expected " << next;
      }
      ++next;
    }
    if (next != emu::writeCalls) {
      return ::testing::AssertionFailure() << next << " success ACKs, " << emu::writeCalls << " writes";
    }
    return ::testing::AssertionSuccess();
  }
};

// (a) Sector 3 written and ACKed, ACK lost; the phone re-sends sector 3, twice.
// Each re-sent packet is refused on its own: 9 x [3, ERR_SECTOR, expected 4].
// No success ACK for sector 3 ever comes again, so the phone must take
// ERR_SECTOR with expected == N+1 as "N is already written".
TEST_F(OtaResendEmu, AckLostResendNackedPerPacketWithNextSectorExpected) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN + 1));  // sector 3's [3, OK] is the lost ACK
  ASSERT_EQ(emu::writeCalls, 4);
  const std::vector<Bytes> pk = packetsOf(img, kN);
  ASSERT_EQ(pk.size(), 9u);

  const std::vector<PacketAck> nack = repeat(9, {kN, kAckSector, kN + 1});
  EXPECT_EQ(play(pk), nack);  // re-send 1
  EXPECT_EQ(emu::writeCalls, 4);
  EXPECT_EQ(currentSectorIndex, kN + 1);
  EXPECT_EQ(sectorBufferLen, 0u);
  EXPECT_EQ(receivedBytes, 4 * kSector);
  EXPECT_EQ(play(pk), nack);  // re-send 2: identical
  EXPECT_EQ(emu::writeCalls, 4);
  EXPECT_TRUE(isOtaInProgress());

  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (b) Last packet (seq 0xFF) of sector 3 lost: no ACK at all. The 8 packets
// that did arrive stay buffered, so re-send 1 lands on top of them: its first
// packet overflows the 4096-byte buffer (ERR_LEN, expected 0, packet dropped,
// buffer cleared) and its last packet fails the CRC (ERR_CRC, expected 0).
// Only re-send 2 succeeds: 3 whole-sector sends in total.
TEST_F(OtaResendEmu, LastPacketLostFirstResendFailsSecondSucceeds) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN));
  const std::vector<Bytes> pk = packetsOf(img, kN);
  EXPECT_TRUE(play(without(pk, 8)).empty());
  EXPECT_EQ(sectorBufferLen, 4000u);
  EXPECT_EQ(emu::writeCalls, 3);

  const std::vector<PacketAck> resend1 = {{0, {kN, kAckLen, 0}}, {8, {kN, kAckCrc, 0}}};
  EXPECT_EQ(play(pk), resend1);
  EXPECT_EQ(emu::writeCalls, 3);
  EXPECT_EQ(sectorBufferLen, 0u);
  EXPECT_EQ(currentSectorIndex, kN);

  const std::vector<PacketAck> resend2 = {{8, {kN, kAckOk, 0}}};
  EXPECT_EQ(play(pk), resend2);
  EXPECT_EQ(emu::writeCalls, 4);

  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (b) with 512-byte packets (7 x 512 + 512): the stale 3584 bytes plus re-sent
// packet 0 fill the buffer exactly, so the overflow comes at packet 1. Same
// outcome: ERR_LEN then ERR_CRC, success on the third send.
TEST_F(OtaResendEmu, LastPacketLostWith512BytePacketsOverflowsOnSecondPacket) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN, 512));
  const std::vector<Bytes> pk = packetsOf(img, kN, 512);
  ASSERT_EQ(pk.size(), 8u);
  EXPECT_TRUE(play(without(pk, 7)).empty());

  const std::vector<PacketAck> resend1 = {{1, {kN, kAckLen, 0}}, {7, {kN, kAckCrc, 0}}};
  EXPECT_EQ(play(pk), resend1);
  const std::vector<PacketAck> resend2 = {{7, {kN, kAckOk, 0}}};
  EXPECT_EQ(play(pk), resend2);

  EXPECT_TRUE(finishFrom(img, kN + 1, 512));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (b) on a short final sector (1000 bytes = 500 + last 500): the stale 500
// bytes plus the re-send fit in the buffer, so re-send 1 gives only ERR_CRC.
// Re-send 2 succeeds and END completes.
TEST_F(OtaResendEmu, LastPacketOfShortFinalSectorLostFirstResendGetsCrcError) {
  const Bytes img = makeImage(5 * kSector + 1000);
  ASSERT_TRUE(startAndSendUpTo(img, 5));
  const std::vector<Bytes> pk = packetsOf(img, 5);
  ASSERT_EQ(pk.size(), 2u);
  EXPECT_TRUE(play(without(pk, 1)).empty());

  const std::vector<PacketAck> resend1 = {{1, {5, kAckCrc, 0}}};
  EXPECT_EQ(play(pk), resend1);
  const std::vector<PacketAck> resend2 = {{1, {5, kAckOk, 0}}};
  EXPECT_EQ(play(pk), resend2);

  EXPECT_TRUE(finishFrom(img, 6));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (c) First packet of sector 3 lost: the last packet's CRC check fails at once
// (ERR_CRC, expected 0) and clears the buffer, so the phone is not left
// waiting. One re-send succeeds. If that success ACK is lost too, a second
// re-send gets 9 x [3, ERR_SECTOR, expected 4] like (a).
TEST_F(OtaResendEmu, FirstPacketLostNackedWithCrcErrorThenOneResendSucceeds) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN));
  const std::vector<Bytes> pk = packetsOf(img, kN);

  const std::vector<PacketAck> lossy = {{7, {kN, kAckCrc, 0}}};
  EXPECT_EQ(play(without(pk, 0)), lossy);
  EXPECT_EQ(sectorBufferLen, 0u);
  EXPECT_EQ(emu::writeCalls, 3);

  const std::vector<PacketAck> resend1 = {{8, {kN, kAckOk, 0}}};
  EXPECT_EQ(play(pk), resend1);
  EXPECT_EQ(play(pk), repeat(9, {kN, kAckSector, kN + 1}));  // its ACK lost: re-send 2
  EXPECT_EQ(emu::writeCalls, 4);

  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (d) A middle packet (index 4) lost: same as (c).
TEST_F(OtaResendEmu, MiddlePacketLostNackedWithCrcErrorThenOneResendSucceeds) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN));
  const std::vector<Bytes> pk = packetsOf(img, kN);

  const std::vector<PacketAck> lossy = {{7, {kN, kAckCrc, 0}}};
  EXPECT_EQ(play(without(pk, 4)), lossy);
  EXPECT_EQ(sectorBufferLen, 0u);

  const std::vector<PacketAck> resend1 = {{8, {kN, kAckOk, 0}}};
  EXPECT_EQ(play(pk), resend1);
  EXPECT_EQ(play(pk), repeat(9, {kN, kAckSector, kN + 1}));
  EXPECT_EQ(emu::writeCalls, 4);

  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (e) Everything the controller sends for sector 3 when the phone re-sends on
// a short timer without reading the ACKs in between: attempt 1 loses its last
// packet (silence), attempt 2 draws ERR_LEN + ERR_CRC, attempt 3 succeeds,
// attempt 4 (sent because the phone took attempt 2's stale errors as attempt
// 3's answer) draws 9 x ERR_SECTOR/expected 4. Exactly one write; the only
// success ACK for sector 3 follows that write. A success ACK for sector N
// always means N's CRC-checked bytes were written once, whichever attempt
// carried them (all attempts carry the same bytes).
TEST_F(OtaResendEmu, AckStreamAcrossFourAttemptsHasOneSuccessForOneWrite) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN));
  const size_t before = fwAcks().size();
  const std::vector<Bytes> pk = packetsOf(img, kN);

  play(without(pk, 8));  // attempt 1
  play(pk);              // attempt 2
  play(pk);              // attempt 3
  play(pk);              // attempt 4

  std::vector<FwAck> expected = {{kN, kAckLen, 0}, {kN, kAckCrc, 0}, {kN, kAckOk, 0}};
  for (int i = 0; i < 9; ++i) expected.push_back({kN, kAckSector, kN + 1});
  const std::vector<FwAck> all = fwAcks();
  EXPECT_EQ(std::vector<FwAck>(all.begin() + static_cast<long>(before), all.end()), expected);
  EXPECT_EQ(emu::writeCalls, 4);

  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (e) The phone moves to sector 4 on the first [3, ERR_SECTOR, 4] while the
// rest of its sector-3 re-send is still queued. On one link they arrive before
// sector 4 and only draw more stale [3, ERR_SECTOR, 4] ACKs (which the phone
// must drop: their sector field is 3). Even if a stale sector-3 packet landed
// in the middle of sector 4, it would only clear the buffer: sector 4 then
// fails its CRC (ERR_CRC) and is re-sent; nothing wrong is written.
TEST_F(OtaResendEmu, StaleResendPacketsBeforeOrInsideNextSectorNeverWriteWrongData) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN + 1));
  const std::vector<Bytes> s3 = packetsOf(img, kN);
  const std::vector<Bytes> s4 = packetsOf(img, kN + 1);

  EXPECT_EQ(play(s3), repeat(9, {kN, kAckSector, kN + 1}));  // queued re-send drains

  std::vector<Bytes> mixed(s4.begin(), s4.begin() + 4);
  mixed.push_back(s3[2]);  // hypothetical stale packet mid-sector
  mixed.insert(mixed.end(), s4.begin() + 4, s4.end());
  const std::vector<PacketAck> acks = {{4, {kN, kAckSector, kN + 1}}, {9, {kN + 1, kAckCrc, 0}}};
  EXPECT_EQ(play(mixed), acks);
  EXPECT_EQ(emu::writeCalls, 4);

  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

// (f) Sector 3's last packet lost, then nothing for OTA_TIMEOUT_MS: the idle
// timer aborts the session and nothing at all is notified on either
// characteristic (the only outward sign is the request for normal connection
// parameters). A late re-send, a probe or END after that get no ACK / 0x0001;
// the phone must START again (full erase) and send from sector 0.
TEST_F(OtaResendEmu, ResendAfterIdleAbortIsSilentAndNeedsFreshStartFromSectorZero) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN));
  const std::vector<Bytes> pk = packetsOf(img, kN);
  play(without(pk, 8));
  const size_t dataNotes = data_->notifications.size();
  const size_t cmdNotes = cmd_->notifications.size();
  emu::events.clear();

  runNotifyTask(OTA_TIMEOUT_MS + 40);
  EXPECT_FALSE(isOtaInProgress());
  EXPECT_EQ(emu::abortCalls, 1);
  EXPECT_EQ(emu::countOf("conn_normal"), 1);
  EXPECT_EQ(data_->notifications.size(), dataNotes);
  EXPECT_EQ(cmd_->notifications.size(), cmdNotes);

  EXPECT_TRUE(play(pk).empty());
  EXPECT_TRUE(probe().empty());
  EXPECT_EQ(emu::writeCalls, 3);
  EXPECT_EQ(end(), 0x0001);

  ASSERT_EQ(start(static_cast<uint32_t>(img.size())), kAckOk);
  EXPECT_EQ(emu::beginCalls, 2);
  EXPECT_EQ(play(pk), repeat(9, {kN, kAckSector, 0}));  // new session expects 0
  EXPECT_TRUE(finishFrom(img, 0, kChunk, /*priorWrites=*/3));
  EXPECT_EQ(emu::writeCalls, 9);  // 3 in the aborted session + 6
}

// A 3-byte data write naming an unreachable sector is answered with
// [0xFFFF, ERR_SECTOR, expected = next sector to send], clears any partial
// sector buffer and counts as activity for the idle timer; it writes nothing.
// Data handling is the same in 8.0, so this works on both.
TEST_F(OtaResendEmu, ProbeReportsExpectedSectorClearsPartialBufferAndKeepsSessionAlive) {
  const Bytes img = makeImage(6 * kSector);
  ASSERT_TRUE(startAndSendUpTo(img, kN));
  const std::vector<Bytes> pk = packetsOf(img, kN);

  // Last packet lost: probe says "still expecting 3" and drops the stale bytes,
  // so a single re-send succeeds (2 sends in total instead of 3).
  play(without(pk, 8));
  ASSERT_EQ(sectorBufferLen, 4000u);
  const std::vector<PacketAck> expecting3 = {{0, {kProbeSector, kAckSector, kN}}};
  EXPECT_EQ(probe(), expecting3);
  EXPECT_EQ(sectorBufferLen, 0u);
  const std::vector<PacketAck> ok = {{8, {kN, kAckOk, 0}}};
  EXPECT_EQ(play(pk), ok);

  // ACK lost: probe says "expecting 4", i.e. sector 3 is already written.
  const std::vector<PacketAck> expecting4 = {{0, {kProbeSector, kAckSector, kN + 1}}};
  EXPECT_EQ(probe(), expecting4);

  // Probes keep the session alive past OTA_TIMEOUT_MS.
  runNotifyTask(OTA_TIMEOUT_MS - 1000);
  EXPECT_EQ(probe(), expecting4);
  runNotifyTask(OTA_TIMEOUT_MS - 1000);
  EXPECT_TRUE(isOtaInProgress());

  EXPECT_EQ(emu::writeCalls, 4);
  EXPECT_TRUE(finishFrom(img, kN + 1));
  EXPECT_TRUE(successAcksMatchWrites());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
