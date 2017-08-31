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

void apply_delta_state(
    float* delta_state, int delta_state_length, unsigned short update_flag,
    float* quat_ISS2B_in, float* gyro_bias_in,
    float* V_B_ISS_ISS_in, float* accel_bias_in, float* P_B_ISS_ISS_in,
    float* ml_quat_ISS2cam_in, float* ml_P_cam_ISS_ISS_in, unsigned short kfl_status_in,
    float* of_quat_ISS2cam_in, float* of_P_cam_ISS_ISS_in,
    float* quat_ISS2B_out, float* gyro_bias_out,
    float* V_B_ISS_ISS_out, float* accel_bias_out, float* P_B_ISS_ISS_out,
    float* ml_quat_ISS2cam_out, float* ml_P_cam_ISS_ISS_out, unsigned short* kfl_status_out,
    float* of_quat_ISS2cam_out, float* of_P_cam_ISS_ISS_out);
