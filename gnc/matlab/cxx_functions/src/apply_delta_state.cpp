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

#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;

void rotate_quat(Quaternionf & q, Vector3f & gibbs_vector, Quaternionf* ret) {
  Quaternionf t(2, gibbs_vector.x(), gibbs_vector.y(), gibbs_vector.z());
  // Eigen's multiply doesn't work the same as simulink...
  // let's keep this to be safe
  ret->x() = q.x() * t.w() + q.w() * t.x() - q.z() * t.y() + q.y() * t.z();
  ret->y() = q.y() * t.w() + q.z() * t.x() + q.w() * t.y() - q.x() * t.z();
  ret->z() = q.z() * t.w() - q.y() * t.x() + q.x() * t.y() + q.w() * t.z();
  ret->w() = q.w() * t.w() - q.x() * t.x() - q.y() * t.y() - q.z() * t.z();
  if (ret->w() < 0) {
    ret->w() = -ret->w(); ret->x() = -ret->x(); ret->y() = -ret->y(); ret->z() = -ret->z();
  }
  // Eigen normalize function doesn't work here, I don't understand why
  if (ret->w() != 0) {
    float mag = sqrt(ret->x() * ret->x() + ret->y() * ret->y() + ret->z() * ret->z() + ret->w() * ret->w());
    ret->x() /= mag; ret->y() /= mag; ret->z() /= mag; ret->w() /= mag;
  }
}

void apply_delta_state(
    float* delta_state, int delta_state_length, unsigned short update_flag,
    float* quat_ISS2B_in, float* gyro_bias_in,
    float* V_B_ISS_ISS_in, float* accel_bias_in, float* P_B_ISS_ISS_in,
    float* ml_quat_ISS2cam_in, float* ml_P_cam_ISS_ISS_in, unsigned short kfl_status_in,
    float* of_quat_ISS2cam_in, float* of_P_cam_ISS_ISS_in,
    float* quat_ISS2B_out, float* gyro_bias_out,
    float* V_B_ISS_ISS_out, float* accel_bias_out, float* P_B_ISS_ISS_out,
    float* ml_quat_ISS2cam_out, float* ml_P_cam_ISS_ISS_out, unsigned short* kfl_status_out,
    float* of_quat_ISS2cam_out, float* of_P_cam_ISS_ISS_out) {
  int num_augs = (delta_state_length - 21) / 6;
  Map<VectorXf> ds(delta_state, delta_state_length);
  Map<Quaternionf> quat(quat_ISS2B_in);
  Map<Quaternionf> out_quat(quat_ISS2B_out);
  Map<Vector3f> gyro_bias(gyro_bias_in);
  Map<Vector3f> out_gyro_bias(gyro_bias_out);
  Map<Vector3f> vel(V_B_ISS_ISS_in);
  Map<Vector3f> out_vel(V_B_ISS_ISS_out);
  Map<Vector3f> accel_bias(accel_bias_in);
  Map<Vector3f> out_accel_bias(accel_bias_out);
  Map<Vector3f> pos(P_B_ISS_ISS_in);
  Map<Vector3f> out_pos(P_B_ISS_ISS_out);
  Map<Quaternionf> ml_quat(ml_quat_ISS2cam_in);
  Map<Quaternionf> out_ml_quat(ml_quat_ISS2cam_out);
  Map<Vector3f> ml_pos(ml_P_cam_ISS_ISS_in);
  Map<Vector3f> out_ml_pos(ml_P_cam_ISS_ISS_out);

  Quaternionf temp_quat1, temp_quat2;
  Vector3f temp_vector = ds.segment<3>(0);
  temp_quat1 = quat;
  rotate_quat(temp_quat1, temp_vector, &temp_quat2);
  out_quat = temp_quat2;
  out_gyro_bias = gyro_bias + ds.segment<3>(3);
  out_vel = vel + ds.segment<3>(6);
  out_accel_bias = accel_bias + ds.segment<3>(9);
  out_pos = pos + ds.segment<3>(12);

  temp_quat1 = ml_quat;
  temp_vector = ds.segment<3>(15);
  rotate_quat(temp_quat1, temp_vector, &temp_quat2);
  out_ml_quat = temp_quat2;
  out_ml_pos = ml_pos + ds.segment<3>(18);

  *kfl_status_out = (kfl_status_in & ~3) | update_flag;

  Map<MatrixXf> of_quat(of_quat_ISS2cam_in, num_augs, 4);
  Map<MatrixXf> out_of_quat(of_quat_ISS2cam_out, num_augs, 4);
  Map<MatrixXf> of_pos(of_P_cam_ISS_ISS_in, num_augs, 3);
  Map<MatrixXf> out_of_pos(of_P_cam_ISS_ISS_out, num_augs, 3);

  for (int i = 0; i < num_augs; i++) {
    Quaternionf t_quat(of_quat(i, 3), of_quat(i, 0), of_quat(i, 1), of_quat(i, 2));
    Quaternionf r;
    temp_vector = ds.segment<3>(21 + 6 * i);
    rotate_quat(t_quat, temp_vector, &r);
    out_of_quat(i, 0) = r.x();
    out_of_quat(i, 1) = r.y();
    out_of_quat(i, 2) = r.z();
    out_of_quat(i, 3) = r.w();
    out_of_pos.row(i) = of_pos.row(i) + ds.segment<3>(24 + 6 * i).transpose();
  }
}

