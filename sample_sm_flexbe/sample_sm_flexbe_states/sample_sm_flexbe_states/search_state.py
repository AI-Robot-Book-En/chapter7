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

"""Search state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

from time import sleep


class SearchState(EventState):
    """
    The `SearchState` represents a behavior in which the robot searches for snacks.
    Based on how many snacks the user has already eaten, the robot decides whether
    to continue searching or stop.

    Outputs
    <= succeeded       Outputs a result indicating that the search has succeeded when snacks are found.
    <= finished        Outputs a result indicating that the search has finished when the user is full.
    <= failed          Outputs a result indicating that the search has failed due to some issue.

    Userdata
    ># eat_counter  int The number of snacks the user has eaten so far (Input)
    ># max_eat      int The number of snacks until the user is full (Input)
    """

    def __init__(self):
        """Define the outcomes and input keys for this state."""
        super().__init__(outcomes=['succeeded', 'finished'],
                         input_keys=['eat_counter', 'max_eat'])

    def execute(self, userdata):
        # Execute the searching process
        sleep(1)
        Logger.loginfo('Searching for snacks') # Log that the robot is in the searching state

        if userdata.eat_counter < userdata.max_eat:
            Logger.loginfo('Snacks found!') # Log when snacks are found

            return 'succeeded' # Return the result 'succeeded'
        else:
            Logger.loginfo('I am already full...') # Log when the user is full

            return 'finished' # Return the result 'finished'
