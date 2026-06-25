#ifndef PTC_TEST_SESSION_H
#define PTC_TEST_SESSION_H

#include <cstdint>
#include <memory>
#include <string>

#include "ptc_client.h"

namespace hesai {
namespace lidar {

class PtcTestSession {
 public:
  PtcTestSession(const std::string &ip, uint16_t port);
  ~PtcTestSession();

  bool Connect();
  const std::string &ip() const { return ip_; }
  uint16_t port() const { return port_; }
  PtcClient &client() { return *client_; }

  void DoReadAll();
  void DoGetReturnMode();
  void DoGetSpinRate();
  void DoGetPtpLockOffset();
  void DoGetPointCloudConfig();
  void DoGetConfigInfoRaw();
  void DoGetPtpDiagnostics();
  void DoGetLidarStatus();
  void DoGetCalibration();

  void DoSetReturnMode();
  void DoSetSpinSpeed();
  void DoSetPtpLockOffset();
  void DoSetPointCloud(bool filter_only, bool ultra_only);

 private:
  std::string HexPreview(const u8Array_t &data, size_t max_bytes = 32) const;
  bool ReadPointCloudConfig(uint8_t &ultra, uint8_t &filter) const;
  void PrintPointCloudConfigLine(uint8_t ultra, uint8_t filter) const;
  void PrintPtpStatusLine(uint8_t ptp_status) const;
  void PrintPtpDiagnosticsFailure(int ret_code, uint8_t ptp_status,
                                  bool have_ptp_status) const;
  void PrintSpinSpeedSetFailure(uint16_t requested_rpm) const;
  bool SetPointCloudSelective(uint8_t ultra, uint8_t filter);

  std::string ip_;
  uint16_t port_;
  std::unique_ptr<PtcClient> client_;
  bool have_point_cloud_cache_;
  uint8_t cached_ultra_;
  uint8_t cached_filter_;
};

}  // namespace lidar
}  // namespace hesai

#endif  // PTC_TEST_SESSION_H
