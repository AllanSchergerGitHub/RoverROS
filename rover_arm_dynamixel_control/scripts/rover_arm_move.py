#! /usr/bin/env python

import time
import rospy
from sensor_msgs.msg import JointState
# from dynamixel_workbench_msgs.srv import JointCommand, JointCommandRequest
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import Header


class RoverArmMove(object):
    def __init__(self):
        rospy.loginfo("RoverArmMove INIT...Please wait.")

        # We subscribe to the joint states to have info of the system

        self.dynamixel_states_topic_name = '/dynamixel_states'
        self._check_dynamixel_states_ready()
        rospy.Subscriber(self.dynamixel_states_topic_name, JointState,
                         self.dynamixel_states_callback)

        # We start the Publisher for the positions of the joints
        self.goal_dynamixel_position_publisher = rospy.Publisher(
            '/goal_dynamixel_position', JointState, queue_size=1)

        # Wait for the service client /joint_command to be running
        dynamixel_command_service_name = "/dynamixel_command"
        rospy.wait_for_service(dynamixel_command_service_name)
        # Create the connection to the service
        self.dynamixel_command_service = rospy.ServiceProxy(dynamixel_command_service_name,
                                                            DynamixelCommand)

        rospy.loginfo("RoverArmMove Ready!")

    def dynamixel_states_callback(self, msg):
        """
        rosmsg show sensor_msgs/DynamixelState
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            string[] name
            float64[] position
            float64[] velocity
            float64[] effort

        :param msg:
        :return:
        """
        self.dynamixel_states_msg = msg

    def _check_dynamixel_states_ready(self):
        self.dynamixel_states_msg = None
        rospy.logdebug("Waiting for " + self.dynamixel_states_topic_name + " to be READY...")
        while self.dynamixel_states_msg is None and not rospy.is_shutdown():
            try:
                self.dynamixel_states_msg = rospy.wait_for_message(
                    self.dynamixel_states_topic_name, JointState, timeout=5.0)
                rospy.logdebug("Current " + self.dynamixel_states_topic_name + " READY=>")

            except Exception:
                rospy.logerr("Current " + self.dynamixel_states_topic_name +
                             " not ready yet, retrying ")

    def move_all_joints(self, position_array):

        rospy.logwarn("move_all_joints STARTED")
        # We check that the position array has the correct number of elements
        number_of_joints = len(self.dynamixel_states_msg.name)

        if len(position_array) == number_of_joints:
            # if self.check_gripper_pos_safe(position_array[6]):
            new_joint_position = JointState()

            h = Header()
            h.stamp = rospy.Time.now()  # Note you need to call rospy.init_node() before this will work
            h.frame_id = self.dynamixel_states_msg.header.frame_id

            new_joint_position.header = h
            new_joint_position.name = self.dynamixel_states_msg.name
            new_joint_position.position = position_array

            # These values arent used, so they dont matter really
            new_joint_position.velocity = self.dynamixel_states_msg.velocity
            new_joint_position.effort = self.dynamixel_states_msg.effort

            rospy.logwarn("PUBLISH STARTED")
            self.goal_dynamixel_position_publisher.publish(new_joint_position)
            rospy.logwarn("PUBLISH FINISHED")
            # else:
            # rospy.logerr("Gripper position NOT valid=" + str(position_array[6]))
        else:
            rospy.logerr("The Array given doesnt have the correct length="+str(number_of_joints))

        rospy.logwarn("move_all_joints FINISHED")

    def move_one_joint(self, cmd, addr, value):
        """
        rossrv show dynamixel_workbench_msgs/DynamixelCommand
            string command

            uint8 id
            string addr_name
            int32 value

            ---

            bool comm_result

        :param joint_id:
        :param position:
        :param units:
        :return:
        """
        dynamixel_cmd = DynamixelCommand()
        dynamixel_cmd.request.command = cmd
        dynamixel_cmd.request.addr_name = addr
        dynamixel_cmd.request.value = value

        # if joint_id == 7:
        #    rospy.logwarn("CHECKING Gripper Value is safe?")
        #    if self.check_gripper_pos_safe(position):

        # Send through the connection the name of the object to be deleted by the service
        #        result = self.joint_command_service(dynamixel_cmd_req)
        #        rospy.logwarn("move_one_joint went ok?="+str(result))
        #    else:
        #        rospy.logwarn("Gripper Value Not safe=" + str(position))
        # else:
        # Send through the connection the name of the object to be deleted by the service
        result = self.joint_command_service(dynamixel_cmd_req)
        rospy.logwarn("move_one_joint went ok?=" + str(result))

    def get_joint_names(self):
        return self.dynamixel_states_msg.name

    def check_gripper_pos_safe(self, gripper_value):
        """
        We need to check that the gripper pos is -1.0 > position[6] > -3.14
        Otherwise it gets jammed
        :param gripper_value:
        :return:
        """
        return (-0.5 > gripper_value > -2.0)


def movement_sequence_test():

    openman_obj = RoverArmMove()

    # NOD
    joint_position_home = [0.08743690699338913, 1.0385050773620605, -2.345456600189209,
                           -0.016873789951205254, -1.4818254709243774]
    joint_position1 = [0.8897088766098022, 0.6059224009513855, -1.4419419765472412,
                       -0.016873789951205254, -1.4818254709243774]
    joint_position2 = [0.8912428617477417, 0.5859806537628174, -1.6060779094696045,
                       -0.016873789951205254, -0.8191457390785217]
    joint_position3 = [0.8897088766098022, 0.6028544902801514, -1.8745245933532715,
                       -0.015339808538556099, 0.5292233824729919]

    # SAY NO
    joint_left = [0.44332045316696167, 1.0630487203598022, -2.345456600189209, 0.5568350553512573,
                  -1.483359456062317]
    joint_right = [-0.20862139761447906, 1.0906603336334229, -2.3071072101593018,
                   -0.6488738656044006, -1.483359456062317]
    joint_middle = [0.0076699042692780495, 1.1274758577346802, -2.325515031814575,
                    0.3344078063964844, -1.4848934412002563]

    # Pendulum
    pend_left = [0.46479618549346924, 0.13345633447170258, -1.728796362876892,
                 1.5907381772994995, -1.6797089576721191]
    pend_middle = [0.39269909262657166, 0.1595340073108673, -2.0984857082366943,
                   -0.09817477315664291, -1.0615147352218628]
    pend_right = [0.006135923322290182, 0.42337870597839355, -1.8806605339050293,
                  -1.306951642036438, -1.0661166906356812]

    joint_position_sequence_nod = []
    joint_position_sequence_nod.append(joint_position_home)
    joint_position_sequence_nod.append(joint_position1)
    joint_position_sequence_nod.append(joint_position2)
    joint_position_sequence_nod.append(joint_position3)
    joint_position_sequence_nod.append(joint_position2)
    joint_position_sequence_nod.append(joint_position3)
    joint_position_sequence_nod.append(joint_position1)
    joint_position_sequence_nod.append(joint_position_home)

    joint_position_sequence_say_no = []
    joint_position_sequence_nod.append(joint_position_home)
    joint_position_sequence_nod.append(joint_left)
    joint_position_sequence_nod.append(joint_middle)
    joint_position_sequence_nod.append(joint_right)
    joint_position_sequence_nod.append(joint_left)
    joint_position_sequence_nod.append(joint_middle)
    joint_position_sequence_nod.append(joint_right)
    joint_position_sequence_nod.append(joint_left)
    joint_position_sequence_nod.append(joint_right)
    joint_position_sequence_nod.append(joint_left)
    joint_position_sequence_nod.append(joint_position_home)

    joint_position_sequence_say_pendulum = []
    joint_position_sequence_say_pendulum.append(joint_position_home)
    joint_position_sequence_say_pendulum.append(pend_left)
    joint_position_sequence_say_pendulum.append(pend_middle)
    joint_position_sequence_say_pendulum.append(pend_right)
    joint_position_sequence_say_pendulum.append(pend_left)
    joint_position_sequence_say_pendulum.append(pend_middle)
    joint_position_sequence_say_pendulum.append(pend_right)
    joint_position_sequence_say_pendulum.append(pend_left)
    joint_position_sequence_say_pendulum.append(pend_middle)
    joint_position_sequence_say_pendulum.append(pend_right)
    joint_position_sequence_say_pendulum.append(joint_position_home)

    for joint_position_array in joint_position_sequence_nod:
        openman_obj.move_all_joints(joint_position_array)
        time.sleep(0.5)

    for joint_position_array in joint_position_sequence_say_no:
        openman_obj.move_all_joints(joint_position_array)
        time.sleep(0.2)

    for joint_position_array in joint_position_sequence_say_pendulum:
        openman_obj.move_all_joints(joint_position_array)
        time.sleep(0.5)


def move_joints_test():
    """
    This is for Geting the positions of the joints without testing them
    live, which is quite dangerous!
    :return:
    """
    openman_obj = RoverArmMove()
    joint_names = openman_obj.get_joint_names()
    rospy.logwarn("Starting Moving Joints GUI...")
    while not rospy.is_shutdown():
        rospy.logwarn("#######"+str(joint_names)+"#####")
        joint_id = int(raw_input("Joint ID="))
        joint_position = float(raw_input("Joint Position Radians="))
        openman_obj.move_one_joint(joint_id, joint_position, unit="rad")
        rospy.logwarn("####################")


if __name__ == "__main__":
    rospy.init_node('rover_arm_move', log_level=rospy.WARN)
    # move_joints_test()
    # movement_sequence_test()
    controller = RoverArmMove()
    controller.move_one_joint()
