# ファイル名：sample_sm.py

import rclpy #[*] Import the module that enables Python to interface with ROS 2.
from rclpy.node import Node
import smach #[*] Import the module for creating state machines.


# Define the search state.
class Search(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['succeeded', 'finished']) #[*] Predefine the possible outcomes of the Search state.
        self.counter = 0 #[*] Variable to count how many times this state has been executed.
        self.logger = _node.get_logger() #[*] Define a logger instance.

    def execute(self, userdata):
        self.logger.info('Searching...') #[*] Log that the robot is in the Search state.
        if self.counter < 3:
            self.logger.info('Found a snack!') #[*] If the state has been executed less than 3 times
            self.counter += 1
            return 'succeeded' #[*] Return the result 'succeeded'.
        else:
            self.logger.info('I am full...') #[*] If the state has been executed 3 times
            return 'finished' #[*] Return the result 'finished'.


# Define the eat state.
class Eat(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['done']) #[*] Predefine the possible outcomes of the Eat state.
        self.logger = _node.get_logger() #[*] Define a logger instance.

    def execute(self, userdata):
        self.logger.info('Eating...') #[*] Log that the robot is in the Eat state.
        return 'done' #[*] Return the result 'done'.


# Define the state machine execution node.
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine') #[*] Register the node with the name state_machine.

    def execute(self):
        # Create the Smach state machine
        sm = smach.StateMachine(outcomes=['end'])
        # Open the container
        with sm: #[*] Define the transitions between states.
            # Add states to the container
            smach.StateMachine.add(
                'SEARCH', Search(self),
                transitions={'succeeded': 'EAT', 'finished': 'end'})
            smach.StateMachine.add(
                'EAT', Eat(self),
                transitions={'done': 'SEARCH'})

        # Smachプランを実行
        outcome = sm.execute()
        self.get_logger().info(f'outcom: {outcome}')


def main():
    rclpy.init() #[*] Initialize ROS 2 communication through rclpy.
    node = StateMachine() #[*] Initialize the state machine node.
    node.execute() #[*] Execute the state machine.
