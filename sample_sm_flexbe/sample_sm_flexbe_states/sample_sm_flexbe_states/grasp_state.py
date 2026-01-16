#!/usr/bin/env python

# Copyright 2024 Keith Valentin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Grasp state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

import random
from time import sleep


class GraspState(EventState):
    """
    The `GraspState` represents a state where the robot attempts to grasp a snack.

    Outputs
    <= succeeded       Indicates that the robot successfully grasped the found snack.
    <= failed          Indicates that the robot failed to grasp the snack due to some issue.
    """

    def __init__(self):
        """Defines the outcomes and input keys for this state."""
        super().__init__(outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        # Execute the grasping process
        sleep(1)  # Pause for 1 second to visualize the process
        Logger.loginfo('Attempting to grasp the snack') # Log that the robot is in the grasping state
        
        prob = random.random() # Generate a random value between [0, 1]
        if 0.5 > prob:
            Logger.loginfo('Successfully grasped the snack!') # Log when the grasp succeeds

            return 'succeeded' # Return the result 'succeeded'
        else:
            Logger.loginfo('Failed to grasp the snack... Trying again!') # Log when the grasp fails

            return 'failed' # Return the result 'failed'
