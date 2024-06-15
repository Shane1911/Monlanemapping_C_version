#include <string>

#include "simulater.h"
using namespace lmap::src;

using namespace std;

void InitLog(char** argv) {
  FLAGS_log_dir = "../data/log/";
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::GLOG_INFO, "../data/log/simulation_INFO_");
  google::SetLogFilenameExtension(".INFO");
  LOG(INFO) << "test begaining!!!";
  return;
}

int main(int argc, char** argv) {
  // build log file.
  InitLog(argv);
  // test fuc
  Simulater simulater;
  if (simulater.DataBuffer()) {
    simulater.InitialProcess();
    simulater.RunDataflow();
  }
  return 0;
}
