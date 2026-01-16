#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Keith Valentin
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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Advanced Sample State Machine.

In this state machine, a new behavior is created by extending the previous
sample to include a grasping state.

Created on Wed Oct 29 2025
@author: Keith Valentin
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from sample_sm_flexbe_states.eat_state import EatState
from sample_sm_flexbe_states.grasp_state import GraspState
from sample_sm_flexbe_states.search_state import SearchState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class AdvancedSampleStateMachineSM(Behavior):
    """
    Define Advanced Sample State Machine.

    In this state machine, a new behavior is created by extending the previous
    sample to include a grasping state.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Advanced Sample State Machine'

        # parameters of this behavior
        self.add_parameter('max_eat', 1)

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:176 y:306, x:172 y:612
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.max_eat = self.max_eat
        _state_machine.userdata.eat_counter = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:320 y:100
            OperatableStateMachine.add('Search',
                                       SearchState(),
                                       transitions={'succeeded': 'Grasp'  # 563 117 -1 -1 -1 -1
                                                    , 'finished': 'finished'  # 259 225 -1 -1 -1 -1
                                                    },
                                       autonomy={'succeeded': Autonomy.Off,
                                                 'finished': Autonomy.Off},
                                       remapping={'eat_counter': 'eat_counter',
                                                  'max_eat': 'max_eat'})

            # x:963 y:97
            OperatableStateMachine.add('Eat',
                                       EatState(),
                                       transitions={'succeeded': 'Search'  # 731 41 -1 -1 -1 -1
                                                    },
                                       autonomy={'succeeded': Autonomy.Off},
                                       remapping={'eat_counter': 'eat_counter'})

            # x:649 y:89
            OperatableStateMachine.add('Grasp',
                                       GraspState(),
                                       transitions={'succeeded': 'Eat'  # 878 109 -1 -1 -1 -1
                                                    , 'failed': 'Grasp'  # 724 188 -1 -1 -1 -1
                                                    },
                                       autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
