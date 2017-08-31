/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

int of_residual_and_h(float* of_measured, float* global_points, float* camera_tf_global, int* valid, int num_points, int ase_of_num_aug,
                       int ase_of_num_features, float tun_ase_mahal_distance_max, float ase_of_r_mag, float ase_inv_focal_length, float ase_distortion, float* P,
                       float* r_out, float* H_out, unsigned int* augs_bitmask, unsigned char* num_of_tracks, float* mahal_dists, float* R_out);
