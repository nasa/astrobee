#ifndef READ_PARAMS_CPP
#define READ_PARAMS_CPP

namespace config_reader {
  class ConfigReader;
}

#include "fam_force_allocation_module.h"
void fam_ReadParams(config_reader::ConfigReader* config,  RT_MODEL_fam_force_allocation_T* fam);

#endif