#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from std_msgs.msg import String
from std_srvs.srv import Empty

import paho.mqtt.client as mqtt

from models_pkgs.srv import vacuumGripper
broker_url = "broker.mqttdashboard.com"
broker_port = 1883
msg = ''


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_ur5_pick_place', anonymous=True)

        self._planning_group = "ur5_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Box added in the path planning
        self._box_name = "box"

        self._boxes = [["box00","box01","box02"],
					   ["box10","box11","box12"],
					   ["box20","box21","box22"]]
        rospy.loginfo(
            '\033[94m' +
            "Planning Group: {}".format(
                self._planning_frame) +
            '\033[0m')
        rospy.loginfo(
            '\033[94m' +
            "End Effector Link: {}".format(
                self._eef_link) +
            '\033[0m')
        rospy.loginfo(
            '\033[94m' +
            "Group Names: {}".format(
                self._group_names) +
            '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=False)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    
    def hard_go_to_pose(self, arg_pose, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    # Function for reaching goal using joint values
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=False)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    # Print current pose values of arm
    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' +
                      "\n" +
                      "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "roll: {}\n".format(roll) +
                      "pitch: {}\n".format(pitch) +
                      "yaw: {}\n".format(yaw) +
                      '\033[0m')

    # Print joint values of arm
    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo(
            '\033[94m' +
            "\nJoint Values: \n\n" +
            "ur5_shoulder_pan_joint: {}\n".format(
                math.degrees(
                    list_joint_values[0])) +
            "ur5_shoulder_lift_joint: {}\n".format(
                math.degrees(
                    list_joint_values[1])) +
            "ur5_elbow_joint: {}\n".format(
                math.degrees(
                    list_joint_values[2])) +
            "ur5_wrist_1_joint: {}\n".format(
                math.degrees(
                    list_joint_values[3])) +
            "ur5_wrist_2_joint: {}\n".format(
                math.degrees(
                    list_joint_values[4])) +
            "ur5_wrist_3_joint: {}\n".format(
                math.degrees(
                    list_joint_values[5])) +
            '\033[0m')

    def vac_gripper(self, r, c, action):
     	    rospy.wait_for_service('/ur5/activate_vacuum_gripper')
     	    try:
     	    	grip_action = rospy.ServiceProxy(
     	            '/ur5/activate_vacuum_gripper', vacuumGripper)
     	    	result = grip_action(action)
     	    	if action:
     	    		self.attach_box(r,c)
     	    	else:
     	    		self.detach_box(r,c)
                        ur5.remove_box(r,c)
     	    	print(result)
     	    except rospy.ServiceException as e:
     	    	print("Service call failed: %s" % e)
 
    def wait_for_state_update(
            self,
            box_is_known=False,
            box_is_attached=False,
            timeout=4):
		box_name = self._boxes[2][2]
		scene = self._scene

		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
		    attached_objects = scene.get_attached_objects([box_name])
		    is_attached = len(attached_objects.keys()) > 0

		    is_known = box_name in scene.get_known_object_names()
		    if (box_is_attached == is_attached) and (box_is_known == is_known):
		        return True

		    rospy.sleep(0.1)
		    seconds = rospy.get_time()

		return False

    def add_box(self, box_poses, timeout=4):
		box_name = self._boxes
		scene = self._scene
		for i in range(0,len(box_poses)):
			row = box_poses[i]
			for j in range(0,len(row)):
				scene.add_box(box_name[i][j], box_poses[i][j], size=(0.15, 0.15, 0.15))
		
    def attach_box(self, r, c, timeout=4):
		box_name = self._boxes[r][c]
		robot = self._robot
		scene = self._scene
		eef_link = self._eef_link
		group_names = self._group_names

		grasping_group = 'ur5_planning_group'
		touch_links = robot.get_link_names(group=grasping_group)
		scene.attach_box(eef_link, box_name, touch_links=touch_links)

		return self.wait_for_state_update(
		    box_is_attached=True,
		    box_is_known=False,
		    timeout=timeout)

    def detach_box(self, r, c, timeout=4):
		box_name = self._boxes[r][c]
		scene = self._scene
		eef_link = self._eef_link
		scene.remove_attached_object(eef_link, name=box_name)
		return self.wait_for_state_update(
		    box_is_known=True,
		    box_is_attached=False,
		    timeout=timeout)

    def remove_box(self, i, j, timeout=4):
		box_name = self._boxes
		scene = self._scene
		
		scene.remove_world_object(box_name[i][j])
		
		return self.wait_for_state_update(
		    box_is_attached=False,
		    box_is_known=False,
		    timeout=timeout)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

ur5 = Ur5Moveit()

def callback_joint_angles(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    angles = [ math.radians(int (i)) for i in data.data.split()]
    print('*********')
    print(angles)
    print('*********')
    
    ur5.hard_set_joint_angles(angles, 5)


def callback_gripper(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    coords = [ int (i) for i in data.data.split()]
    print('*********')
    print(coords)
    print('*********')
    ur5.vac_gripper(coords[0],coords[1],coords[2])

def main():
    global ur5

 
    # -58 -81 33 -132 -122 0 BOX00
    #  69 -20 21    0    4 0



	# Default:		  0 0 0 0 0 0
	# Bin    Red:	-14 0 0 0 0 0
	# Bin  Green:    70 0 0 0 0 0
	# Bin Yellow:   154 0 0 0 0 0

	#  -53  -84   43 -139 -128 0 BOX00
	# -125 -104   62 -138  -56 0 BOX01
	# -160  -86   46 -141  -22 0 BOX02
	#  -53  -90   60   27  128 0 BOX11
	#  120  -62 -112   -6   59 0 BOX11
	#   56  -82 -100    1  122 0 BOX12
	#  -54  -95  102   -7  127 0 BOX20
	# -118 -118  119    0   63 0 BOX21
	# -159 -100  107   -7   22 0 BOX22
    
    box_pose00 = geometry_msgs.msg.PoseStamped()
    box_pose00.header.frame_id = "world"
    box_pose00.pose.position.x = 0.28
    box_pose00.pose.position.y = -0.42
    box_pose00.pose.position.z = 1.87
    box_pose00.pose.orientation.x = -0.000002
    box_pose00.pose.orientation.y = 0
    box_pose00.pose.orientation.z = 0
    box_pose00.pose.orientation.w = 0
    
    box_pose01 = geometry_msgs.msg.PoseStamped()
    box_pose01.header.frame_id = "world"
    box_pose01.pose.position.x = 0.0
    box_pose01.pose.position.y = -0.42
    box_pose01.pose.position.z = 1.87
    box_pose01.pose.orientation.x = -0.000002
    box_pose01.pose.orientation.y = 0
    box_pose01.pose.orientation.z = 0
    box_pose01.pose.orientation.w = 0
    
    box_pose02 = geometry_msgs.msg.PoseStamped()
    box_pose02.header.frame_id = "world"
    box_pose02.pose.position.x = -0.28
    box_pose02.pose.position.y = -0.42
    box_pose02.pose.position.z = 1.87
    box_pose02.pose.orientation.x = -0.000002
    box_pose02.pose.orientation.y = 0
    box_pose02.pose.orientation.z = 0
    box_pose02.pose.orientation.w = 0
    
    box_pose10 = geometry_msgs.msg.PoseStamped()
    box_pose10.header.frame_id = "world"
    box_pose10.pose.position.x = 0.28
    box_pose10.pose.position.y = -0.42
    box_pose10.pose.position.z = 1.59
    box_pose10.pose.orientation.x = -0.000002
    box_pose10.pose.orientation.y = 0
    box_pose10.pose.orientation.z = 0
    box_pose10.pose.orientation.w = 0
    
    box_pose11 = geometry_msgs.msg.PoseStamped()
    box_pose11.header.frame_id = "world"
    box_pose11.pose.position.x = 0.0
    box_pose11.pose.position.y = -0.42
    box_pose11.pose.position.z = 1.59
    box_pose11.pose.orientation.x = -0.000002
    box_pose11.pose.orientation.y = 0
    box_pose11.pose.orientation.z = 0
    box_pose11.pose.orientation.w = 0
    
    box_pose12 = geometry_msgs.msg.PoseStamped()
    box_pose12.header.frame_id = "world"
    box_pose12.pose.position.x = -0.28
    box_pose12.pose.position.y = -0.42
    box_pose12.pose.position.z = 1.59
    box_pose12.pose.orientation.x = -0.000002
    box_pose12.pose.orientation.y = 0
    box_pose12.pose.orientation.z = 0
    box_pose12.pose.orientation.w = 0
    
    box_pose20 = geometry_msgs.msg.PoseStamped()
    box_pose20.header.frame_id = "world"
    box_pose20.pose.position.x = 0.28
    box_pose20.pose.position.y = -0.42
    box_pose20.pose.position.z = 1.36
    box_pose20.pose.orientation.x = -0.000002
    box_pose20.pose.orientation.y = 0
    box_pose20.pose.orientation.z = 0
    box_pose20.pose.orientation.w = 0
    
    box_pose21 = geometry_msgs.msg.PoseStamped()
    box_pose21.header.frame_id = "world"
    box_pose21.pose.position.x = 0.0
    box_pose21.pose.position.y = -0.42
    box_pose21.pose.position.z = 1.36
    box_pose21.pose.orientation.x = -0.000002
    box_pose21.pose.orientation.y = 0
    box_pose21.pose.orientation.z = 0
    box_pose21.pose.orientation.w = 0
    
    box_pose22 = geometry_msgs.msg.PoseStamped()
    box_pose22.header.frame_id = "world"
    box_pose22.pose.position.x = -0.28
    box_pose22.pose.position.y = -0.42
    box_pose22.pose.position.z = 1.36
    box_pose22.pose.orientation.x = -0.000002
    box_pose22.pose.orientation.y = 0
    box_pose22.pose.orientation.z = 0
    box_pose22.pose.orientation.w = 0
    

    box_poses = [[box_pose00,box_pose01,box_pose02],
				 [box_pose10,box_pose11,box_pose12],
				 [box_pose20,box_pose21,box_pose22]]

    ur5.add_box(box_poses)

    while not rospy.is_shutdown():
        rospy.Subscriber("joint_angles", String, callback_joint_angles)
        rospy.Subscriber("vaccum", String, callback_gripper)
        rospy.spin()

	
    del ur5



if __name__ == '__main__':
    main()
