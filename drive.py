#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist, Vector3 
from kobuki_msgs.msg import BumperEvent

# Global variables
pub = None
keyboard_command = Twist()
is_bumped = False
path_plan_command = Twist()

def update_on_keyboard_input(key_msg):
    """
    Callback for keyboard input. Updates the global keyboard command
    and sets the KEY parameter to True if a command is received.
    
    :param key_msg: The Twist message representing the keyboard input command.
    """
    global keyboard_command
    keyboard_command = key_msg
    if not is_command_empty(keyboard_command):
        rospy.set_param("KEY", True)

def check_bumper_status(bumper_msg):
    """
    Callback for bumper status. Sets the global is_bumped variable to True 
    if the bumper is pressed, and schedules a reset after 10 seconds.
    
    :param bumper_msg: The BumperEvent message.
    """
    global is_bumped
    if bumper_msg.state == 1 and not is_bumped: 
        is_bumped = True
        rospy.Timer(rospy.Duration(secs=10), reset_bump_status, oneshot=True)  # Schedule a reset after 10 seconds

def reset_bump_status(timer_event):
    """
    Reset the bump status after the specified duration (10 seconds in this case).
    
    :param timer_event: The timer event triggering the callback.
    """
    global is_bumped
    is_bumped = False

def process_and_publish_commands(timer_event):
    """
    Evaluates the priority of commands and publishes the appropriate
    command to the robot.
    
    :param timer_event: The timer event triggering the callback.
    """
    if is_bumped:  # Highest priority: Halt if bumper is pressed
        rospy.set_param("HALT", True)
        stop_movement_command = Twist(Vector3(0,0,0), Vector3(0,0,0))
        pub.publish(stop_movement_command)
    elif not is_command_empty(keyboard_command):  # Next priority: keyboard commands
        pub.publish(keyboard_command)
        rospy.set_param("KEY", False)
    else:  # Default action
        pub.publish(path_plan_command)

def is_command_empty(twist_command):
    """
    Checks if the given Twist command is empty (all values are 0).
    
    :param twist_command: The Twist command to check.
    :return: True if the command is empty, False otherwise.
    """
    state_linear = all(val == 0 for val in [twist_command.linear.x, twist_command.linear.y, twist_command.linear.z])
    state_angular = all(val == 0 for val in [twist_command.angular.x, twist_command.angular.y, twist_command.angular.z])
    return state_linear and state_angular

def main():
    """
    The main function initializes the node, sets up publishers and subscribers,
    and spins the node to keep it running.
    """
    global pub

    rospy.init_node("drive_node", anonymous=True)
    pub = rospy.Publisher("/mobile_base/commands/velocity",Twist, queue_size=1)

    rospy.Subscriber("/project1/keyboard_input", Twist, update_on_keyboard_input)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, check_bumper_status)

    # Initialize parameters
    rospy.set_param("HALT", False)
    rospy.set_param("WAIT", False)
    rospy.set_param("KEY", False)

    # Setup timer to publish commands at regular intervals
    rospy.Timer(rospy.Duration(secs=.2), process_and_publish_commands)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
