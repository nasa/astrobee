#ifndef READ_PARAMS_CPP
#define READ_PARAMS_CPP

namespace config_reader {
  class ConfigReader;
}

#include "ctl_controller0.h"
void ctl_ReadParams(config_reader::ConfigReader* config,  RT_MODEL_ctl_controller0_T* ctl);

#endif