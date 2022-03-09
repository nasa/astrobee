% Copyright (c) 2017, United States Government, as represented by the
% Administrator of the National Aeronautics and Space Administration.
%
% All rights reserved.
%
% The Astrobee platform is licensed under the Apache License, Version 2.0
% (the "License"); you may not use this file except in compliance with the
% License. You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
% License for the specific language governing permissions and limitations
% under the License.

function [hr_global_landmarks, hr_P_hr_ISS_ISS, hr_quat_ISS2hr] = ...
            compute_hr_tf_and_landmark(ml_P_cam_ISS_ISS, ml_quat_ISS2cam, ...
                                        hr_P_hr_cam_cam, hr_quat_cam2hr, ...
                                        hr_feature_observations, hr_feature_valid, hr_update_global_pose_flag)    

    persistent hr_P_hr_ISS_ISS_pers;
    persistent hr_quat_ISS2hr_pers;    
            
    if isempty(hr_P_hr_ISS_ISS_pers)
        hr_P_hr_ISS_ISS_pers = ml_P_cam_ISS_ISS + quaternion_to_rotation(ml_quat_ISS2cam)' * hr_P_hr_cam_cam;
    end
    if isempty(hr_quat_ISS2hr_pers)
        hr_quat_ISS2hr_pers = quatmult(ml_quat_ISS2cam', hr_quat_cam2hr')';
    end    
                            
    if hr_update_global_pose_flag == 1
        hr_P_hr_ISS_ISS_pers = ml_P_cam_ISS_ISS + quaternion_to_rotation(ml_quat_ISS2cam)' * hr_P_hr_cam_cam;
        hr_quat_ISS2hr_pers = quatmult(ml_quat_ISS2cam', hr_quat_cam2hr')';        
    end            
    m = size(hr_feature_observations, 1);
    hr_global_landmarks = single(zeros(m, 3));
    for j=1:m
        if hr_feature_valid(j) == 0            
            break;
        end 
        ph = quaternion_to_rotation(hr_quat_cam2hr) * (hr_feature_observations(j,:)' - hr_P_hr_cam_cam);
        hr_global_landmarks(j,:) = hr_P_hr_ISS_ISS_pers + quaternion_to_rotation(hr_quat_ISS2hr_pers)' * ph;
    end    
    hr_P_hr_ISS_ISS = hr_P_hr_ISS_ISS_pers;
    hr_quat_ISS2hr = hr_quat_ISS2hr_pers;
       