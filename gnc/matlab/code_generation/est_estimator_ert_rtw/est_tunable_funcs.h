#ifndef READ_PARAMS_CPP
#define READ_PARAMS_CPP

namespace config_reader {
  class ConfigReader;
}

#include "est_estimator.h"
void est_ReadParams(config_reader::ConfigReader* config,  RT_MODEL_est_estimator_T* est);

#endif