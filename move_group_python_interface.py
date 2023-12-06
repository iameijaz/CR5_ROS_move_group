#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "cr5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, home_pos=False):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        if(home_pos):
        	joint_goal[0] = 0
        	joint_goal[1] = 0
        	joint_goal[2] = 0
        	joint_goal[3] = 0
        	joint_goal[4] = 0
        	joint_goal[5] = 0
        else:
        	joint_goal[0] = -2.9367 
        	joint_goal[1] = -0.4508
        	joint_goal[2] = -1.2933
        	joint_goal[3] = 0.1699
        	joint_goal[4] = 1.5341
        	joint_goal[5] = 0.1852


        move_group.go(joint_goal, wait=True)
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0 # -0.0142
        pose_goal.position.x = 0.4 # -0.6999
        pose_goal.position.y = 0.1 # 0.00886
        pose_goal.position.z = 0.4 # 0.36444

        move_group.set_pose_target(pose_goal)


        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.y -= 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))



        #wpose = move_group.get_current_pose().pose
        wpose.position.y -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= 0.1
        #wpose.orientation.w = 0.5 ###
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.05, 0.0 # waypoints to follow  # eef_step 0.01
        )  # jump_threshold
        return plan, fraction



    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail



def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the CR5 Layup Trial")
        print("----------------------------------------------------------")
        print("")

        tutorial = MoveGroupPythonInterfaceTutorial()
        #tutorial.go_to_joint_state(home_pos=True)
        #input(
        #    "============ Press `Enter' when ready ..."
        #)
        #tutorial.go_to_joint_state()
        #input("============ Press `Enter` to see the simulated path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        tutorial.display_trajectory(cartesian_plan)
        input("============ Press `Enter` to execute that path ...")
        tutorial.execute_plan(cartesian_plan)
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
