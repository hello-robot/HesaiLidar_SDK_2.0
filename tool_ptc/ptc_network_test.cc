#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "logger.h"
#include "ptc_client.h"

using namespace hesai::lidar;

namespace {

constexpr size_t kConfigDestIpOffset = 12;
constexpr size_t kConfigDestUdpPortOffset = 16;
constexpr size_t kConfigMinLen = 18;

const char *kRdkDestIp = "255.255.255.255";
const char *kRdkNetMask = "255.255.255.0";
const char *kRdkGateway = "192.168.1.1";
constexpr uint16_t kDefaultPtcPort = 9347;
constexpr uint16_t kVlanIdInvalid = 4095;

std::string FormatIpv4FromConfigBytes(const u8Array_t &config, size_t offset)
{
  if (config.size() < offset + 4) {
    return "unknown";
  }
  return std::to_string(static_cast<unsigned>(config[offset + 3])) + "." +
         std::to_string(static_cast<unsigned>(config[offset + 2])) + "." +
         std::to_string(static_cast<unsigned>(config[offset + 1])) + "." +
         std::to_string(static_cast<unsigned>(config[offset + 0]));
}

bool ParseDestUdpPort(const u8Array_t &config, uint16_t &port)
{
  if (config.size() < kConfigDestUdpPortOffset + 2) {
    return false;
  }
  port = static_cast<uint16_t>(
      (static_cast<uint16_t>(config[kConfigDestUdpPortOffset]) << 8) |
      config[kConfigDestUdpPortOffset + 1]);
  return true;
}

bool ReadConfigSummary(PtcClient &client, std::string &device_ip,
                       std::string &dest_ip, uint16_t &dest_udp_port)
{
  u8Array_t config;
  if (client.GetConfigInfoRaw(config) != 0 || config.size() < kConfigMinLen) {
    return false;
  }
  device_ip = FormatIpv4FromConfigBytes(config, 0);
  dest_ip = FormatIpv4FromConfigBytes(config, kConfigDestIpOffset);
  return ParseDestUdpPort(config, dest_udp_port);
}

bool WaitForOpen(PtcClient &client, const std::string &ip, uint16_t port)
{
  std::cout << "Connecting PTC to " << ip << ":" << port << "...\n";
  while (!client.IsOpen()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Connected.\n";
  return true;
}

void PrintUsage(const char *prog)
{
  std::cerr
      << "Usage: " << prog
      << " <current_ip> <new_device_ip> <dest_udp_port> [ptc_tcp_port]\n"
      << "  Example (RDK left): " << prog << " 192.168.1.201 192.168.1.202 2378\n"
      << "  Default ptc_tcp_port: " << kDefaultPtcPort << "\n";
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 4 || argc > 5) {
    PrintUsage(argv[0]);
    return 1;
  }

  Logger::GetInstance().setLogTargetRule(LOGTARGET::HESAI_LOG_TARGET_CONSOLE);
  Logger::GetInstance().setLogLevelRule(
      LOGLEVEL::HESAI_LOG_WARNING | LOGLEVEL::HESAI_LOG_ERROR |
      LOGLEVEL::HESAI_LOG_FATAL);

  const std::string current_ip = argv[1];
  const std::string new_device_ip = argv[2];
  const uint16_t dest_udp_port = static_cast<uint16_t>(std::atoi(argv[3]));
  const uint16_t ptc_port = (argc == 5)
                                ? static_cast<uint16_t>(std::atoi(argv[4]))
                                : kDefaultPtcPort;

  std::cout << "============================================================\n"
            << "WARNING: ONE LIDAR ONLY — network config test tool\n"
            << "This writes destination UDP + device IP to the connected lidar.\n"
            << "Network changes take effect after a power cycle.\n"
            << "============================================================\n\n";

  std::cout << "Plan:\n"
            << "  connect at:     " << current_ip << ":" << ptc_port << "\n"
            << "  new device IP:  " << new_device_ip << "\n"
            << "  dest IP:        " << kRdkDestIp << "\n"
            << "  dest UDP port:  " << dest_udp_port << "\n\n";

  PtcClient client(current_ip, ptc_port);
  if (!WaitForOpen(client, current_ip, ptc_port)) {
    return 1;
  }

  std::string device_ip;
  std::string dest_ip;
  uint16_t read_udp_port = 0;
  if (ReadConfigSummary(client, device_ip, dest_ip, read_udp_port)) {
    std::cout << "Before SET:\n"
              << "  device IP:      " << device_ip << "\n"
              << "  dest IP:        " << dest_ip << "\n"
              << "  dest UDP port:  " << read_udp_port << "\n\n";
  } else {
    std::cout << "Before SET: GET config info failed\n\n";
  }

  std::cout << "SET destination IP/port (PTC 0x20)...\n";
  if (!client.SetDesIpandPort(kRdkDestIp, dest_udp_port, 0)) {
    std::cerr << "SetDesIpandPort FAILED (ret_code=" << client.ret_code_ << ")\n";
    return 1;
  }
  std::cout << "SetDesIpandPort OK\n";

  std::cout << "SET device network (PTC 0x21)...\n";
  if (!client.SetNet(new_device_ip, kRdkNetMask, kRdkGateway, 0,
                     kVlanIdInvalid)) {
    std::cerr << "SetNet FAILED (ret_code=" << client.ret_code_ << ")\n";
    return 1;
  }
  std::cout << "SetNet OK\n\n";

  std::cout << "Power-cycle the lidar now (disconnect power, reconnect).\n"
            << "Press Enter when done...";
  std::cout.flush();
  std::string line;
  std::getline(std::cin, line);

  PtcClient verify_client(new_device_ip, ptc_port);
  if (!WaitForOpen(verify_client, new_device_ip, ptc_port)) {
    std::cerr << "Could not connect at " << new_device_ip << ":" << ptc_port
              << " after power cycle.\n";
    return 1;
  }

  if (!ReadConfigSummary(verify_client, device_ip, dest_ip, read_udp_port)) {
    std::cerr << "After power cycle: GET config info failed\n";
    return 1;
  }

  std::cout << "\nAfter power cycle:\n"
            << "  device IP:      " << device_ip << "\n"
            << "  dest IP:        " << dest_ip << "\n"
            << "  dest UDP port:  " << read_udp_port << "\n\n";

  const bool ip_ok = (device_ip == new_device_ip);
  const bool udp_ok = (read_udp_port == dest_udp_port);
  const bool dest_ip_ok = (dest_ip == kRdkDestIp);

  std::cout << "Verify device IP:      "
            << (ip_ok ? "PASS" : "FAIL") << " (expected " << new_device_ip
            << ")\n";
  std::cout << "Verify dest IP:          "
            << (dest_ip_ok ? "PASS" : "FAIL") << " (expected " << kRdkDestIp
            << ")\n";
  std::cout << "Verify dest UDP port:    "
            << (udp_ok ? "PASS" : "FAIL") << " (expected " << dest_udp_port
            << ")\n";

  if (ip_ok && udp_ok && dest_ip_ok) {
    std::cout << "\nOverall: PASS\n";
    return 0;
  }
  std::cout << "\nOverall: FAIL\n";
  return 1;
}
