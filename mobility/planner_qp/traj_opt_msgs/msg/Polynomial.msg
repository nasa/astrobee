# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Polynomial message used by the QP planner.

# Implemented Basis Choices
uint8 STANDARD=0
uint8 LEGENDRE=1
uint8 BEZIER=2 # Same as BSpline and Bernstein modulo marginalization
uint8 ENDPOINT=3
uint8 CHEBYSHEV=4

# Future Basis Choices
uint8 SPECIAL=5
uint8 FUN=6

# There should be 1 more coeffs than the degree
int32 degree
float32 dt
uint8 basis
float32[] coeffs