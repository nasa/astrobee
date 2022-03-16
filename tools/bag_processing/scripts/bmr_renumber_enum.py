#!/usr/bin/env python
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

"""
This file provides utility functions to be used in *.bmr bag migration
rules to help with migrating legacy messages that contain an enumerated
field where the label numbering has changed.

We should stop renumbering enumerated types in message definitions! See
[1].

Example usage is in [2].

[1] https://github.com/nasa/astrobee/blob/develop/doc/general_documentation/maintaining_telemetry.md
[2] https://github.com/nasa/astrobee/blob/develop/communications/ff_msgs/bmr/2021_08_25_MotionResult_renumber_enum.bmr
"""

from __future__ import print_function

import logging
import re

ENUM_REGEX = re.compile(
    r"^\s* (?P<type>\w+) \s* (?P<label>\w+) \s* = \s* (?P<value>[\+\-1-9\.]+)",
    re.VERBOSE,
)


def int_or_float(s):
    try:
        return int(s)
    except ValueError:
        return float(s)


def field_enum_from_msg_text(msg_text):
    field_enum = {}
    for line in msg_text.splitlines():
        m = ENUM_REGEX.search(line)
        if not m:
            continue
        label = m.group("label")
        value = int_or_float(m.group("value"))
        field_enum[label] = value
    return field_enum


def get_renumber_rule(field_enum_old, field_enum_new, rename_labels=None):
    if rename_labels is None:
        rename_labels = {}

    enum_mapping = {}
    missing_labels = []
    for old_label, old_value in field_enum_old.items():
        new_label = rename_labels.get(old_label, old_label)
        new_value = field_enum_new.get(new_label)
        if new_value is None:
            missing_labels.append(new_label)
        else:
            if old_value != new_value:
                enum_mapping[old_value] = new_value
    return (enum_mapping, missing_labels)


def get_renumber_function(
    msg_text_old, msg_text_new, rename_labels=None, verbose=False
):
    field_enum_old = field_enum_from_msg_text(msg_text_old)
    field_enum_new = field_enum_from_msg_text(msg_text_new)
    enum_mapping, missing_labels = get_renumber_rule(
        field_enum_old, field_enum_new, rename_labels=rename_labels
    )
    if verbose:
        logging.info("enum_mapping: %s", enum_mapping)
        logging.info("missing_labels: %s", missing_labels)
    assert not missing_labels
    return lambda old_value: enum_mapping[old_value]


######################################################################
# TEST
######################################################################


def _test():
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    # these calls should work
    get_renumber_function(TEST_MSG_ENUM_OLD, TEST_MSG_ENUM_NEW, verbose=True)
    get_renumber_function(
        TEST_MSG_ENUM_NEW,
        TEST_MSG_ENUM_OLD,
        verbose=True,
        rename_labels={
            "TOLERANCE_VIOLATION_POSITION_ENDPOINT": "TOLERANCE_VIOLATION_POSITION"
        },
    )

    # this last call should raise an assertion error due to the missing label
    get_renumber_function(TEST_MSG_ENUM_NEW, TEST_MSG_ENUM_OLD, verbose=True)


TEST_MSG_ENUM_OLD = """
int32 ALREADY_THERE                =   2  # MOVE: We are already at the location
int32 SUCCESS                      =   1  # ALL: Motion succeeded
int32 PREEMPTED                    =   0  # ALL: Motion preempted by thirdparty
int32 PLAN_FAILED                  =  -1  # MOVE/EXEC: Plan/bootstrap failed
int32 VALIDATE_FAILED              =  -2  # MOVE/EXEC: No comms with mapper
int32 PMC_FAILED                   =  -3  # MOVE/EXEC: PMC failed
int32 CONTROL_FAILED               =  -4  # ALL: Control failed
int32 OBSTACLE_DETECTED            =  -5  # ALL: Obstacle / replan disabled
int32 REPLAN_NOT_ENOUGH_TIME       =  -6  # MOVE/EXEC: Not enough time to replan
int32 REPLAN_FAILED                =  -7  # MOVE/EXEC: Replanning failed
int32 REVALIDATE_FAILED            =  -8  # MOVE/EXEC: Revalidating failed
int32 NOT_IN_WAITING_MODE          =  -9  # ALL: Internal failure
int32 INVALID_FLIGHT_MODE          =  -10 # ALL: No flight mode specified
int32 UNEXPECTED_EMPTY_SEGMENT     =  -11 # EXEC: Segment empty
int32 COULD_NOT_RESAMPLE           =  -12 # EXEC: Could not resample segment
int32 UNEXPECTED_EMPTY_STATES      =  -13 # MOVE: State vector empty
int32 INVALID_COMMAND              =  -14 # Command rejected
int32 CANNOT_QUERY_ROBOT_POSE      =  -15 # TF2 failed to find the current pose
int32 NOT_ON_FIRST_POSE            =  -16 # EXEC: Not on first pose of exec
int32 BAD_DESIRED_VELOCITY         =  -17 # Requested vel too high
int32 BAD_DESIRED_ACCELERATION     =  -18 # Requested accel too high
int32 BAD_DESIRED_OMEGA            =  -19 # Requested omega too high
int32 BAD_DESIRED_ALPHA            =  -20 # Requested alpha too high
int32 BAD_DESIRED_RATE             =  -21 # Requested rate too low
int32 BAD_DESIRED_VELOCITY         =  -17 # Requested vel too high
int32 BAD_DESIRED_ACCELERATION     =  -18 # Requested accel too high
int32 BAD_DESIRED_OMEGA            =  -19 # Requested omega too high
int32 BAD_DESIRED_ALPHA            =  -20 # Requested alpha too high
int32 BAD_DESIRED_RATE             =  -21 # Requested rate too low
int32 TOLERANCE_VIOLATION_POSITION =  -22 # Position tolerance violated
int32 TOLERANCE_VIOLATION_ATTITUDE =  -23 # Attitude tolerance violated
int32 TOLERANCE_VIOLATION_VELOCITY =  -24 # Velocity tolerance violated
int32 TOLERANCE_VIOLATION_OMEGA    =  -25 # Omega tolerance violated
int32 VIOLATES_RESAMPLING          =  -26 # Validation: could not resample@10Hz
int32 VIOLATES_KEEP_OUT            =  -27 # Validation: Keep out violation
int32 VIOLATES_KEEP_IN             =  -28 # Validation: Keep in violation
int32 VIOLATES_MINIMUM_FREQUENCY   =  -29 # Validation: Sample frequency too low
int32 VIOLATES_STATIONARY_ENDPOINT =  -30 # Validation: Last setpoint not static
int32 VIOLATES_FIRST_IN_PAST       =  -31 # Validation: First timestamp in past
int32 VIOLATES_MINIMUM_SETPOINTS   =  -32 # Validation: Not enough setpoints
int32 VIOLATES_HARD_LIMIT_VEL      =  -33 # Validation: Velocity too high
int32 VIOLATES_HARD_LIMIT_ACCEL    =  -34 # Validation: Acceleration too high
int32 VIOLATES_HARD_LIMIT_OMEGA    =  -35 # Validation: Omega too high
int32 VIOLATES_HARD_LIMIT_ALPHA    =  -36 # Validation: Alpha too high
int32 CANCELLED                    =  -37 # ALL: Motion cancelled by callee
int32 INVALID_REFERENCE_FRAME      =  -38 # ALL: Unknown reference frame
"""

TEST_MSG_ENUM_NEW = """
int32 ALREADY_THERE                         =   2  # MOVE: We are already at the location
int32 SUCCESS                               =   1  # ALL: Motion succeeded
int32 PREEMPTED                             =   0  # ALL: Motion preempted by thirdparty
int32 PLAN_FAILED                           =  -1  # MOVE/EXEC: Plan/bootstrap failed
int32 VALIDATE_FAILED                       =  -2  # MOVE/EXEC: No comms with mapper
int32 PMC_FAILED                            =  -3  # MOVE/EXEC: PMC failed
int32 CONTROL_FAILED                        =  -4  # ALL: Control failed
int32 OBSTACLE_DETECTED                     =  -5  # ALL: Obstacle / replan disabled
int32 REPLAN_NOT_ENOUGH_TIME                =  -6  # MOVE/EXEC: Not enough time to replan
int32 REPLAN_FAILED                         =  -7  # MOVE/EXEC: Replanning failed
int32 REVALIDATE_FAILED                     =  -8  # MOVE/EXEC: Revalidating failed
int32 NOT_IN_WAITING_MODE                   =  -9  # ALL: Internal failure
int32 INVALID_FLIGHT_MODE                   =  -10 # ALL: No flight mode specified
int32 UNEXPECTED_EMPTY_SEGMENT              =  -11 # EXEC: Segment empty
int32 COULD_NOT_RESAMPLE                    =  -12 # EXEC: Could not resample segment
int32 UNEXPECTED_EMPTY_STATES               =  -13 # MOVE: State vector empty
int32 INVALID_COMMAND                       =  -14 # Command rejected
int32 CANNOT_QUERY_ROBOT_POSE               =  -15 # TF2 failed to find the current pose
int32 NOT_ON_FIRST_POSE                     =  -16 # EXEC: Not on first pose of exec
int32 BAD_DESIRED_VELOCITY                  =  -17 # Requested vel too high
int32 BAD_DESIRED_ACCELERATION              =  -18 # Requested accel too high
int32 BAD_DESIRED_OMEGA                     =  -19 # Requested omega too high
int32 BAD_DESIRED_ALPHA                     =  -20 # Requested alpha too high
int32 BAD_DESIRED_RATE                      =  -21 # Requested rate too low
int32 TOLERANCE_VIOLATION_POSITION_ENDPOINT =  -22 # Position tolerance violated
int32 TOLERANCE_VIOLATION_POSITION          =  -23 # Position tolerance violated
int32 TOLERANCE_VIOLATION_ATTITUDE          =  -24 # Attitude tolerance violated
int32 TOLERANCE_VIOLATION_VELOCITY          =  -25 # Velocity tolerance violated
int32 TOLERANCE_VIOLATION_OMEGA             =  -26 # Omega tolerance violated
int32 VIOLATES_RESAMPLING                   =  -27 # Validation: could not resample@10Hz
int32 VIOLATES_KEEP_OUT                     =  -28 # Validation: Keep out violation
int32 VIOLATES_KEEP_IN                      =  -29 # Validation: Keep in violation
int32 VIOLATES_MINIMUM_FREQUENCY            =  -30 # Validation: Sample frequency too low
int32 VIOLATES_STATIONARY_ENDPOINT          =  -31 # Validation: Last setpoint not static
int32 VIOLATES_FIRST_IN_PAST                =  -32 # Validation: First timestamp in past
int32 VIOLATES_MINIMUM_SETPOINTS            =  -33 # Validation: Not enough setpoints
int32 VIOLATES_HARD_LIMIT_VEL               =  -34 # Validation: Velocity too high
int32 VIOLATES_HARD_LIMIT_ACCEL             =  -35 # Validation: Acceleration too high
int32 VIOLATES_HARD_LIMIT_OMEGA             =  -36 # Validation: Omega too high
int32 VIOLATES_HARD_LIMIT_ALPHA             =  -37 # Validation: Alpha too high
int32 CANCELLED                             =  -38 # ALL: Motion cancelled by callee
int32 INVALID_REFERENCE_FRAME               =  -39 # ALL: Unknown reference frame
"""
