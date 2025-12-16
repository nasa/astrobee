#ifndef DAQP_CODEGEN_H
#define DAQP_CODEGEN_H

#include <stdio.h>
#include "types.h"

void render_daqp_workspace(DAQPWorkspace* work, const char *fname, const char* dir);

void write_daqp_workspace_h(FILE *f, DAQPWorkspace *work);
void write_daqp_workspace_src(FILE *f, DAQPWorkspace *work);
void write_daqp_settings_src(FILE*  f, DAQPSettings* settings);
void write_daqp_bnb_h(FILE*  f, DAQPBnB* bnb, const int n);
void write_daqp_bnb_src(FILE*  f, DAQPBnB* bnb, const int n);

void write_float_array(FILE *f, c_float* a, const int N, const char *name);
void write_int_array(FILE *f, int* a, const int N, const char *name);

#endif //ifndef DAQP_CODEGEN_H
