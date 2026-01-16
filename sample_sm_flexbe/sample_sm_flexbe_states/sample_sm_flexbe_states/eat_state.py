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

"""Eat state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

from time import sleep


class EatState(EventState):
    """
    The `EatState` represents a state where the robot aims to eat a snack that was found in the previous state.
    It determines randomly whether to eat or not, regardless of how many snacks the user has already eaten.

    Outputs
    <= succeeded       Indicates that the Eat state has finished execution.

    Userdata
    ># eat_counter  int The number of snacks the user has eaten so far (Input)
    #> eat_counter  int Updates and outputs the number of snacks eaten (Output)
    """

    def __init__(self):
        """Defines the outcomes and userdata keys for this state."""
        super().__init__(outcomes=['succeeded'],
                         input_keys=['eat_counter'],
                         output_keys=['eat_counter'])

    def execute(self, userdata):
        # Execute the eating process
        sleep(1)
        Logger.loginfo('Eating one piece of snack!') # Log the eating action
        userdata.eat_counter += 1 # Update eat_counter
        Logger.loginfo('I have eaten {} pieces of snack so far!'.format(userdata.eat_counter)) # Log the current count of snacks

        return 'succeeded' # Return the result 'succeeded'
