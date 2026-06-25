#include "ptc_menu.h"

#include "ptc_client.h"
#include <cctype>
#include <iostream>
#include <limits>
#include <string>

namespace hesai {
namespace lidar {

void PtcMenu::PrintMainMenu(const std::string &ip, uint16_t port)
{
  std::cout << "\n--- PTC menu (" << ip << ":" << port << ") ---\n"
            << "  GET\n"
            << "    1  read_all\n"
            << "    2  get_return_mode\n"
            << "    3  get_spin_rate\n"
            << "    4  get_ptp_lock_offset\n"
            << "    5  get_point_cloud_config (GET 0x122 live ultra + filter)\n"
            << "    6  get_config_info_raw (len + head hex)\n"
            << "    7  get_ptp_diagnostics\n"
            << "    8  get_lidar_status\n"
            << "    9  get_calibration (size only)\n"
            << "  SET\n"
            << "   10  set_return_mode        -> prompt mode (required)\n"
            << "   11  set_spin_speed         -> prompt 600 or 1200 RPM (default 600)\n"
            << "   12  set_ptp_lock_offset    -> prompt 1-1000 us (e.g. 350 for Stretch FAB)\n"
            << "   13  set_point_cloud        -> prompt ultra + filter (each: value or 'none')\n"
            << "   14  set_filter_only        -> prompt filter (ultra = keep)\n"
            << "   15  set_ultra_precise_only -> prompt ultra (filter = keep)\n"
            << "    0  quit\n"
            << "Choice: ";
}

int PtcMenu::ReadChoice()
{
  int choice = -1;
  if (!(std::cin >> choice)) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return -1;
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return choice;
}

bool PtcMenu::ReadPointCloudField(const char *field_name, uint8_t &value,
                                  bool &is_none)
{
  std::cout << field_name << " [0-3 ultra / 0-2 filter, or none]: ";
  std::string line;
  if (!std::getline(std::cin, line)) {
    return false;
  }
  std::string trimmed = line;
  trimmed.erase(trimmed.begin(),
                std::find_if(trimmed.begin(), trimmed.end(),
                             [](unsigned char ch) { return !std::isspace(ch); }));
  trimmed.erase(
      std::find_if(trimmed.rbegin(), trimmed.rend(),
                   [](unsigned char ch) { return !std::isspace(ch); })
          .base(),
      trimmed.end());
  if (trimmed.empty()) {
    std::cout << "No input.\n";
    return false;
  }
  for (auto &ch : trimmed) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (trimmed == "none") {
    is_none = true;
    value = kPointCloudKeepCurrent;
    return true;
  }
  try {
    int parsed = std::stoi(trimmed);
    if (parsed < 0 || parsed > 255) {
      std::cout << "Value out of range.\n";
      return false;
    }
    is_none = false;
    value = static_cast<uint8_t>(parsed);
    return true;
  } catch (...) {
    std::cout << "Invalid input.\n";
    return false;
  }
}

int PtcMenu::ReadInt(const char *prompt)
{
  std::cout << prompt;
  int value = -1;
  if (!(std::cin >> value)) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return -1;
  }
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return value;
}

uint16_t PtcMenu::ReadPtpLockOffsetUs()
{
  std::cout << "PTP lock offset us [" << kPtpLockOffsetMinUs << "-"
            << kPtpLockOffsetMaxUs << ", default 350]: ";
  std::string line;
  if (!std::getline(std::cin, line)) {
    return 0;
  }
  if (line.empty()) {
    return 350;
  }
  try {
    int parsed = std::stoi(line);
    if (parsed < kPtpLockOffsetMinUs || parsed > kPtpLockOffsetMaxUs) {
      std::cout << "Value must be in [" << kPtpLockOffsetMinUs << ", "
                << kPtpLockOffsetMaxUs << "].\n";
      return 0;
    }
    return static_cast<uint16_t>(parsed);
  } catch (...) {
    std::cout << "Invalid input.\n";
    return 0;
  }
}

uint16_t PtcMenu::ReadSpinRateRpm()
{
  std::cout << "spin speed RPM [" << kSpinRateRpm600 << " or "
            << kSpinRateRpm1200 << ", default " << kSpinRateRpmDefault
            << "]: ";
  std::string line;
  if (!std::getline(std::cin, line)) {
    return 0;
  }
  if (line.empty()) {
    return kSpinRateRpmDefault;
  }
  try {
    const int parsed = std::stoi(line);
    if (!IsValidSpinRateRpm(static_cast<uint32_t>(parsed))) {
      std::cout << "Spin rate must be " << kSpinRateRpm600 << " or "
                << kSpinRateRpm1200 << " RPM.\n";
      return 0;
    }
    return static_cast<uint16_t>(parsed);
  } catch (...) {
    std::cout << "Invalid input.\n";
    return 0;
  }
}

}  // namespace lidar
}  // namespace hesai
