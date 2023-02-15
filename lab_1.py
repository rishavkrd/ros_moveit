#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_commander


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def startle_callback(data):
    """
    Callback that implements the startle behavior.
    """
    if detect_noise(data):
        execute_behavior()
        

def isNoise(arr):
	thr = 400
	sum = 0
	for i in arr:
		sum+=abs(i)
	if(abs(sum)>thr):
		return True
	print(abs(sum))
	return False
	
def detect_noise(data):
    """
    The perceptual schema.
    Args:
        data: audio data.
    Return:
        bool: alert
    """
    THRESHOLD = 0.5
    chunk = 1024
    alert = False
    ##################
    # YOUR CODE HERE #
    ##################
    if (data):
    	
    	
    	#aud = stream.read(chunk)
    	# check level against threshold, you'll have to write getLevel()
    	if isNoise(data.data):
        	alert = True
        	print("Clap Detected")
    

    return alert


def execute_behavior():
    """
    The motor schema.
    Args:
        alert (bool): ALERT signal.
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    

    group_name = "survivor_buddy_head"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)
    
    # robot:
    print("============ Printing robot state")
    #print(robot.get_current_state())
    #print("")

    # joint value planning
    joint_goal = move_group.get_current_joint_values()
    original_goal = move_group.get_current_joint_values()
    ##################################
    # YOUR CODE HERE                 #
    # You may modify the lines below #
    ##################################
    joint_goal[0] =  0 # Enter a value
    joint_goal[1] =  .2 # Enter a value
    joint_goal[2] =  0 # Enter a value
    joint_goal[3] =  .05 # Enter a value
    
    #pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.w = 1.0
    #pose_goal.position.x = 0.4
    #pose_goal.position.y = 0.1
    #pose_goal.position.z = 0.4

    #move_group.set_pose_target(pose_goal)
    

    move_group.go(joint_goal, wait=True)
    plan = move_group.plan()
    move_group.stop()

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    pub.publish(display_trajectory)

    # execute plan
    move_group.execute(plan[1], wait=True)
    
    # Back to previous state
    
    move_group.go(original_goal, wait=True)
    plan = move_group.plan()
    move_group.stop()

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    pub.publish(display_trajectory)

    # execute plan
    move_group.execute(plan[1], wait=True)
    return True


if __name__ == "__main__":
    rospy.init_node("lab_1_node", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

    pub = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )
    sub = rospy.Subscriber("/audio", Float32MultiArray, callback=startle_callback)
    rospy.loginfo("Node started.")

    rospy.spin()
