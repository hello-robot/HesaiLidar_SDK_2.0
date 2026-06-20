#ifndef PTC_MENU_H
#define PTC_MENU_H

#include <cstdint>
#include <string>

namespace hesai {
namespace lidar {

class PtcMenu {
 public:
  static void PrintMainMenu(const std::string &ip, uint16_t port);
  static int ReadChoice();
  static bool ReadPointCloudField(const char *field_name, uint8_t &value,
                                  bool &is_none);
  static int ReadInt(const char *prompt);
  static uint16_t ReadPtpLockOffsetUs();
  static uint16_t ReadSpinRateRpm();
};

}  // namespace lidar
}  // namespace hesai

#endif  // PTC_MENU_H
