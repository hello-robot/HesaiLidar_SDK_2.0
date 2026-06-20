#include <cstdlib>
#include <iostream>

#include "logger.h"
#include "ptc_menu.h"
#include "ptc_test_session.h"

using namespace hesai::lidar;

int main(int argc, char **argv)
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <ip> <port>\n";
    return 1;
  }

  Logger::GetInstance().setLogTargetRule(LOGTARGET::HESAI_LOG_TARGET_CONSOLE);
  Logger::GetInstance().setLogLevelRule(
      LOGLEVEL::HESAI_LOG_WARNING | LOGLEVEL::HESAI_LOG_ERROR |
      LOGLEVEL::HESAI_LOG_FATAL);

  const std::string ip = argv[1];
  const uint16_t port = static_cast<uint16_t>(std::atoi(argv[2]));
  PtcTestSession session(ip, port);
  if (!session.Connect()) {
    return 1;
  }

  while (true) {
    PtcMenu::PrintMainMenu(ip, port);
    const int choice = PtcMenu::ReadChoice();
    switch (choice) {
      case 0:
        std::cout << "Bye.\n";
        return 0;
      case 1:
        session.DoReadAll();
        break;
      case 2:
        session.DoGetReturnMode();
        break;
      case 3:
        session.DoGetSpinRate();
        break;
      case 4:
        session.DoGetPtpLockOffset();
        break;
      case 5:
        session.DoGetPointCloudConfig();
        break;
      case 6:
        session.DoGetConfigInfoRaw();
        break;
      case 7:
        session.DoGetPtpDiagnostics();
        break;
      case 8:
        session.DoGetLidarStatus();
        break;
      case 9:
        session.DoGetCalibration();
        break;
      case 10:
        session.DoSetReturnMode();
        break;
      case 11:
        session.DoSetSpinSpeed();
        break;
      case 12:
        session.DoSetPtpLockOffset();
        break;
      case 13:
        session.DoSetPointCloud(false, false);
        break;
      case 14:
        session.DoSetPointCloud(true, false);
        break;
      case 15:
        session.DoSetPointCloud(false, true);
        break;
      default:
        std::cout << "Unknown choice.\n";
        break;
    }
  }
}
