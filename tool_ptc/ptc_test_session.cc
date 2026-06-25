#include "ptc_test_session.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "ptc_jt128_labels.h"
#include "ptc_menu.h"

namespace hesai {
namespace lidar {
namespace {

constexpr size_t kConfigReturnModeOffset = 32;
constexpr size_t kConfigDestUdpPortOffset = 16;
constexpr size_t kConfigMinLen = 33;

}  // namespace

PtcTestSession::PtcTestSession(const std::string &ip, uint16_t port)
    : ip_(ip),
      port_(port),
      client_(nullptr),
      have_point_cloud_cache_(false),
      cached_ultra_(0),
      cached_filter_(0) {}

PtcTestSession::~PtcTestSession() = default;

bool PtcTestSession::Connect()
{
  client_ = std::make_unique<PtcClient>(ip_, port_);
  std::cout << "Waiting for PTC connection to " << ip_ << ":" << port_
            << "...\n";
  while (!client_->IsOpen()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Connected.\n";
  return true;
}

std::string PtcTestSession::HexPreview(const u8Array_t &data,
                                       size_t max_bytes) const
{
  std::ostringstream oss;
  const size_t n = std::min(data.size(), max_bytes);
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < n; ++i) {
    oss << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  if (data.size() > max_bytes) {
    oss << "...";
  }
  return oss.str();
}

bool PtcTestSession::ReadPointCloudConfig(uint8_t &ultra, uint8_t &filter) const
{
  return client_->GetPointCloudConfig(ultra, filter);
}

void PtcTestSession::PrintPointCloudConfigLine(uint8_t ultra,
                                             uint8_t filter) const
{
  std::cout << "point cloud:    "
            << ptc_labels::FormatPointCloudPair(ultra, filter) << "\n";
}

void PtcTestSession::PrintPtpStatusLine(uint8_t ptp_status) const
{
  std::cout << "ptp_status:     "
            << ptc_labels::FormatPtpStatus(ptp_status) << "\n";
}

void PtcTestSession::PrintPtpDiagnosticsFailure(int ret_code,
                                              uint8_t ptp_status,
                                              bool have_ptp_status) const
{
  std::cout << "get_ptp_diagnostics: cmd 0x06 FAILED (ret_code=" << ret_code
            << ")\n";
  if (have_ptp_status) {
    PrintPtpStatusLine(ptp_status);
  }
  if (ret_code == 5 && have_ptp_status && ptp_status == 0) {
    std::cout << "note: PTP is Free run — master offset diagnostics require "
                 "Tracking or Locked PTP\n";
  } else if (have_ptp_status && ptp_status != 2) {
    std::cout << "note: offset diagnostics usually need PTP Locked; use menu 8 "
                 "for status\n";
  }
}

bool PtcTestSession::SetPointCloudSelective(uint8_t ultra, uint8_t filter)
{
  uint8_t current_ultra = 0;
  uint8_t current_filter = 0;
  if (!ReadPointCloudConfig(current_ultra, current_filter)) {
    std::cout << "current config: FAILED to read (GET 0x122)\n";
    return false;
  }
  std::cout << "current config: "
            << ptc_labels::FormatPointCloudPair(current_ultra, current_filter)
            << "\n";

  const uint8_t sent_ultra =
      (ultra == kPointCloudKeepCurrent) ? current_ultra : ultra;
  const uint8_t sent_filter =
      (filter == kPointCloudKeepCurrent) ? current_filter : filter;

  const bool ok = client_->SetPointCloudConfigSelective(
      ultra, filter, have_point_cloud_cache_, cached_ultra_, cached_filter_);
  std::cout << "sent (0x121):   "
            << ptc_labels::FormatPointCloudPair(sent_ultra, sent_filter)
            << "\n";
  if (!ok) {
    std::cout << "result:         SET FAILED (ret_code=" << client_->ret_code_
              << ")\n";
    return false;
  }

  std::cout << "result:         SET OK (ret_code=" << client_->ret_code_ << ")\n";

  uint8_t read_ultra = 0;
  uint8_t read_filter = 0;
  if (ReadPointCloudConfig(read_ultra, read_filter)) {
    std::cout << "readback (0x122): "
              << ptc_labels::FormatPointCloudPair(read_ultra, read_filter)
              << " "
              << (read_ultra == sent_ultra && read_filter == sent_filter
                      ? "MATCH"
                      : "MISMATCH")
              << "\n";
    cached_ultra_ = read_ultra;
    cached_filter_ = read_filter;
    have_point_cloud_cache_ = true;
  } else {
    std::cout << "readback (0x122): FAILED\n";
    cached_ultra_ = sent_ultra;
    cached_filter_ = sent_filter;
    have_point_cloud_cache_ = true;
  }

  u8Array_t raw;
  if (client_->GetPointCloudConfigRaw(raw) == 0) {
    std::cout << "GET 0x122 raw:  " << HexPreview(raw, raw.size()) << "\n";
  }
  return true;
}

void PtcTestSession::DoReadAll()
{
  std::cout << "\n=== read_all ===\n";
  u8Array_t config;
  if (client_->GetConfigInfoRaw(config) == 0 && config.size() >= kConfigMinLen) {
    std::cout << "device IP:      " << static_cast<unsigned>(config[3]) << "."
              << static_cast<unsigned>(config[2]) << "."
              << static_cast<unsigned>(config[1]) << "."
              << static_cast<unsigned>(config[0]) << "\n";
    if (config.size() >= kConfigDestUdpPortOffset + 2) {
      const uint16_t dest_port = static_cast<uint16_t>(
          (static_cast<uint16_t>(config[kConfigDestUdpPortOffset]) << 8) |
          config[kConfigDestUdpPortOffset + 1]);
      std::cout << "dest UDP port:  " << dest_port << "\n";
    }
    std::cout << "config raw len: " << config.size() << "\n";
  } else {
    std::cout << "config info:    FAILED\n";
  }

  uint8_t return_mode = 0;
  if (client_->GetReturnMode(return_mode)) {
    std::cout << "return mode:    " << static_cast<unsigned>(return_mode)
              << " (" << ptc_labels::ReturnModeLabel(return_mode) << ")\n";
  } else {
    std::cout << "return mode:    FAILED\n";
  }

  uint16_t spin_rate = 0;
  if (client_->GetSpinRate(spin_rate)) {
    std::cout << "spin rate:      " << spin_rate << " RPM\n";
  } else {
    std::cout << "spin rate:      FAILED\n";
  }

  uint16_t ptp_offset = 0;
  if (client_->GetPTPLockOffset(ptp_offset) == 0) {
    std::cout << "ptp lock offset:" << ptp_offset << " us\n";
  } else {
    std::cout << "ptp lock offset:FAILED\n";
  }

  uint8_t ultra = 0;
  uint8_t filt = 0;
  if (ReadPointCloudConfig(ultra, filt)) {
    PrintPointCloudConfigLine(ultra, filt);
  } else {
    std::cout << "point cloud:    FAILED (GET 0x122)\n";
  }

  uint8_t ptp_status = 0;
  if (client_->GetLidarPtpStatus(ptp_status)) {
    PrintPtpStatusLine(ptp_status);
  } else {
    std::cout << "ptp_status:     FAILED\n";
  }

  if (ptp_status == 0) {
    std::cout << "ptp diagnostics: skipped (PTP Free run)\n";
  } else {
    u8Array_t ptp_diag;
    if (client_->GetPTPDiagnostics(ptp_diag, 1) == 0 && !ptp_diag.empty()) {
      std::cout << "ptp diagnostics len: " << ptp_diag.size() << "\n";
      if (ptp_diag.size() >= 8) {
        const int64_t offset_ns =
            (static_cast<int64_t>(ptp_diag[0]) << 56) |
            (static_cast<int64_t>(ptp_diag[1]) << 48) |
            (static_cast<int64_t>(ptp_diag[2]) << 40) |
            (static_cast<int64_t>(ptp_diag[3]) << 32) |
            (static_cast<int64_t>(ptp_diag[4]) << 24) |
            (static_cast<int64_t>(ptp_diag[5]) << 16) |
            (static_cast<int64_t>(ptp_diag[6]) << 8) |
            static_cast<int64_t>(ptp_diag[7]);
        std::cout << "ptp offset:     " << std::abs(offset_ns / 1000.0)
                  << " us (" << offset_ns << " ns)\n";
      }
    } else {
      std::cout << "ptp diagnostics: unavailable (ret_code="
                << client_->ret_code_ << ")\n";
    }
  }
}

void PtcTestSession::DoGetReturnMode()
{
  uint8_t return_mode = 0;
  if (!client_->GetReturnMode(return_mode)) {
    std::cout << "get_return_mode: FAILED\n";
    return;
  }
  std::cout << "return_mode: " << static_cast<unsigned>(return_mode) << " ("
            << ptc_labels::ReturnModeLabel(return_mode) << ")\n";
}

void PtcTestSession::DoGetSpinRate()
{
  uint16_t spin_rate = 0;
  if (!client_->GetSpinRate(spin_rate)) {
    std::cout << "get_spin_rate: FAILED\n";
    return;
  }
  std::cout << "spin_rate: " << spin_rate << " RPM\n";
}

void PtcTestSession::DoGetPtpLockOffset()
{
  uint16_t offset_us = 0;
  if (client_->GetPTPLockOffset(offset_us) != 0) {
    std::cout << "get_ptp_lock_offset: FAILED\n";
    return;
  }
  std::cout << "ptp_lock_offset: " << offset_us << " us\n";
}

void PtcTestSession::DoGetPointCloudConfig()
{
  u8Array_t raw;
  if (client_->GetPointCloudConfigRaw(raw) != 0) {
    std::cout << "get_point_cloud_config: GET 0x122 FAILED\n";
    return;
  }
  std::cout << "GET 0x122 raw: " << HexPreview(raw, raw.size()) << "\n";

  uint8_t ultra = 0;
  uint8_t filt = 0;
  if (ReadPointCloudConfig(ultra, filt)) {
    PrintPointCloudConfigLine(ultra, filt);
  } else {
    std::cout << "parse: FAILED (expected at least "
              << kPointCloudLiveConfigMinLen << " bytes)\n";
  }
}

void PtcTestSession::DoGetConfigInfoRaw()
{
  u8Array_t raw;
  if (client_->GetConfigInfoRaw(raw) != 0) {
    std::cout << "get_config_info_raw: FAILED\n";
    return;
  }
  std::cout << "config_info len: " << raw.size() << "\n";
  std::cout << "config_info head: " << HexPreview(raw, 32) << "\n";
}

void PtcTestSession::DoGetPtpDiagnostics()
{
  uint8_t ptp_status = 0;
  const bool have_ptp_status = client_->GetLidarPtpStatus(ptp_status);

  u8Array_t raw;
  if (client_->GetPTPDiagnostics(raw, 1) != 0) {
    PrintPtpDiagnosticsFailure(client_->ret_code_, ptp_status, have_ptp_status);
    return;
  }

  std::cout << "ptp_diagnostics len: " << raw.size() << "\n";
  if (raw.size() != kPtpDiagnosticsPayloadLen) {
    std::cout << "note: expected " << kPtpDiagnosticsPayloadLen
              << " bytes, got " << raw.size() << "\n";
  }
  if (raw.size() >= 8) {
    const int64_t offset_ns =
        (static_cast<int64_t>(raw[0]) << 56) |
        (static_cast<int64_t>(raw[1]) << 48) |
        (static_cast<int64_t>(raw[2]) << 40) |
        (static_cast<int64_t>(raw[3]) << 32) |
        (static_cast<int64_t>(raw[4]) << 24) |
        (static_cast<int64_t>(raw[5]) << 16) |
        (static_cast<int64_t>(raw[6]) << 8) | static_cast<int64_t>(raw[7]);
    std::cout << "offset_ns:      " << offset_ns << "\n";
    std::cout << "offset_us:      " << std::abs(offset_ns / 1000.0) << "\n";
  }
  if (raw.size() > 8) {
    std::cout << "payload head:   " << HexPreview(raw, 32) << "\n";
  }
  if (have_ptp_status) {
    PrintPtpStatusLine(ptp_status);
  }
}

void PtcTestSession::DoGetLidarStatus()
{
  if (client_->GetLidarStatus() != 0) {
    std::cout << "get_lidar_status: FAILED\n";
  }
}

void PtcTestSession::DoGetCalibration()
{
  u8Array_t raw;
  if (client_->GetCorrectionInfo(raw) != 0) {
    std::cout << "get_calibration: FAILED\n";
    return;
  }
  std::cout << "calibration size: " << raw.size() << " bytes\n";
}

void PtcTestSession::PrintSpinSpeedSetFailure(uint16_t requested_rpm) const
{
  std::cout << "set_spin_speed: FAILED\n";
  std::cout << "  allowed:        " << kSpinRateRpm600
            << " RPM (default), " << kSpinRateRpm1200 << " RPM\n";
  std::cout << "  requested:      " << requested_rpm << " RPM\n";
  std::cout << "  command:        PTC 0x17 SET_SPIN_SPEED\n";
  uint16_t current_rpm = 0;
  if (client_->GetSpinRate(current_rpm)) {
    std::cout << "  current config: " << current_rpm << " RPM\n";
  }
  if (client_->ret_code_ > 0) {
    std::cout << "  lidar ret_code: " << client_->ret_code_
              << " (non-zero = rejected by firmware)\n";
  } else if (!IsValidSpinRateRpm(requested_rpm)) {
    std::cout << "  reason:         invalid RPM for JT128\n";
  }
}

void PtcTestSession::DoSetReturnMode()
{
  const int mode = PtcMenu::ReadInt("return mode [0-5]: ");
  if (mode < 0 || mode > 255) {
    std::cout << "Invalid return mode.\n";
    return;
  }
  const uint8_t value = static_cast<uint8_t>(mode);
  if (!client_->SetReturnMode(value)) {
    std::cout << "set_return_mode: FAILED (ret_code=" << client_->ret_code_
              << ")\n";
    return;
  }
  std::cout << "set_return_mode: SET OK\n";
  uint8_t readback = 0;
  if (client_->GetReturnMode(readback)) {
    std::cout << "readback: " << static_cast<unsigned>(readback) << " ("
              << ptc_labels::ReturnModeLabel(readback) << ") "
              << (readback == value ? "MATCH" : "MISMATCH") << "\n";
  }
}

void PtcTestSession::DoSetSpinSpeed()
{
  const uint16_t rpm = PtcMenu::ReadSpinRateRpm();
  if (rpm == 0) {
    return;
  }
  if (!client_->SetSpinSpeed(rpm)) {
    PrintSpinSpeedSetFailure(rpm);
    return;
  }
  std::cout << "set_spin_speed: SET OK (" << rpm << " RPM)\n";
  uint16_t readback = 0;
  if (client_->GetSpinRate(readback)) {
    std::cout << "readback: " << readback << " RPM "
              << (readback == rpm ? "MATCH" : "MISMATCH") << "\n";
  }
}

void PtcTestSession::DoSetPtpLockOffset()
{
  const uint16_t offset_us = PtcMenu::ReadPtpLockOffsetUs();
  if (offset_us == 0) {
    return;
  }
  if (!client_->SetPTPLockOffset(offset_us)) {
    std::cout << "set_ptp_lock_offset: FAILED (ret_code=" << client_->ret_code_
              << ")\n";
    return;
  }
  std::cout << "set_ptp_lock_offset: SET OK\n";
  uint16_t readback = 0;
  if (client_->GetPTPLockOffset(readback) == 0) {
    std::cout << "readback: " << readback << " us "
              << (readback == offset_us ? "MATCH" : "MISMATCH") << "\n";
  }
}

void PtcTestSession::DoSetPointCloud(bool filter_only, bool ultra_only)
{
  uint8_t ultra = kPointCloudKeepCurrent;
  uint8_t filter = kPointCloudKeepCurrent;
  bool ultra_none = false;
  bool filter_none = false;

  if (ultra_only) {
    if (!PtcMenu::ReadPointCloudField("ultra_precise", ultra, ultra_none)) {
      return;
    }
    if (ultra_none) {
      std::cout << "ultra_precise is required.\n";
      return;
    }
    filter = kPointCloudKeepCurrent;
  } else if (filter_only) {
    if (!PtcMenu::ReadPointCloudField("filter", filter, filter_none)) {
      return;
    }
    if (filter_none) {
      std::cout << "filter is required.\n";
      return;
    }
    ultra = kPointCloudKeepCurrent;
  } else {
    if (!PtcMenu::ReadPointCloudField("ultra_precise", ultra, ultra_none)) {
      return;
    }
    if (!PtcMenu::ReadPointCloudField("filter", filter, filter_none)) {
      return;
    }
    if (ultra_none && filter_none) {
      std::cout << "At least one of ultra_precise or filter must be provided.\n";
      return;
    }
  }

  SetPointCloudSelective(ultra, filter);
}

}  // namespace lidar
}  // namespace hesai
