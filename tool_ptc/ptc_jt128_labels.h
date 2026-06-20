#ifndef PTC_JT128_LABELS_H
#define PTC_JT128_LABELS_H

#include <cstdint>
#include <string>

namespace hesai {
namespace lidar {
namespace ptc_labels {

inline const char *UltraPreciseLabel(uint8_t value)
{
  switch (value) {
    case 0: return "Low";
    case 1: return "Medium";
    case 2: return "Strong";
    case 3: return "OFF";
    default: return "unknown";
  }
}

inline const char *FilterLabel(uint8_t value)
{
  switch (value) {
    case 0: return "Disabled";
    case 1: return "Medium";
    case 2: return "Strong";
    default: return "unknown";
  }
}

inline const char *ReturnModeLabel(uint8_t value)
{
  switch (value) {
    case 0: return "last";
    case 1: return "strongest";
    case 2: return "last_and_strongest";
    case 3: return "first";
    case 4: return "last_and_first";
    case 5: return "first_and_strongest";
    default: return "unknown";
  }
}

inline const char *PtpStatusLabel(uint8_t value)
{
  switch (value) {
    case 0: return "Free run";
    case 1: return "Tracking";
    case 2: return "Locked";
    case 3: return "Frozen";
    default: return "unknown";
  }
}

inline std::string FormatPtpStatus(uint8_t value)
{
  return std::to_string(value) + " (" + PtpStatusLabel(value) + ")";
}

inline std::string FormatPointCloudPair(uint8_t ultra, uint8_t filter)
{
  return std::string("ultra=") + std::to_string(ultra) + " (" +
         UltraPreciseLabel(ultra) + ") filter=" + std::to_string(filter) +
         " (" + FilterLabel(filter) + ")";
}

}  // namespace ptc_labels
}  // namespace lidar
}  // namespace hesai

#endif  // PTC_JT128_LABELS_H
