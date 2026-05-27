#include "hesai_lidar_sdk.hpp"
#include <fstream>
#include <thread>
#include <chrono>
#include <cstdlib> // for getenv
#include <filesystem> // for std::filesystem::create_directories


int main(int argc, char *argv[])
{
  HesaiLidarSdk<LidarPointXYZICRT> sample;
  DriverParam param;

  // assign param
  param.use_gpu = false;
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.input_param.device_ip_address = "192.168.1.201";
  param.input_param.udp_port = 2368;
  param.input_param.is_use_ptc = true;
  param.input_param.ptc_port = 9347;
  param.input_param.pcap_path = "Your pcap file path";
  param.input_param.correction_file_path = "Your correction file path";
  param.input_param.firetimes_path = "Your firetime file path";

  param.decoder_param.distance_correction_flag = false;
  param.decoder_param.socket_buffer_size = 262144000;

  //init lidar with param
  sample.Init(param);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  //Print out lidar config
  sample.lidar_ptr_->ptc_client_->GetConfigInfo();

//   //Print out lidar status
//   sample.lidar_ptr_->ptc_client_->GetLidarStatus();

  //Save correction info
  u8Array_t sData;
  if (sample.lidar_ptr_->ptc_client_->GetCorrectionInfo(sData) != 0) {
    printf("FAIL: Can't download lidar calibration");
    return -1;
  }
  std::string base_path = std::getenv("HELLO_FLEET_PATH");
  std::string fleet_id  = std::getenv("HELLO_FLEET_ID");
  std::string save_dir = base_path + "/" + fleet_id + "/calibration_hesais";
  std::filesystem::create_directories(save_dir);
  std::string file_path = save_dir + "/right_lidar_calibration.dat";
  std::ofstream fout(file_path, std::ios::out | std::ios::binary);
  if (!fout) {
    printf("FAIL: Can't open file for writing lidar calibration out");
    return -1;
  }
  fout.write(reinterpret_cast<const char*>(sData.data()), sData.size());
  fout.close();
  printf("Saved calibration to %s (%zu bytes)\n", file_path.c_str(), sData.size());

  //Set return mode to strongest
  sample.lidar_ptr_->ptc_client_->SetReturnMode(1); // 1 -> "Strongest" only. Should yield around 115200 points per frame.
  printf("Set return mode to strongest\n");

  printf("Done. Exiting...\n");
  return 0;
}
