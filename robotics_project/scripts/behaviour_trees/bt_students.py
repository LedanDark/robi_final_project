#!/usr/bin/env python
import py_trees as pt, py_trees_ros as ptr, rospy
from actionlib import SimpleActionClient
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import PoseArray, Pose, Point, Vector3, Quaternion
import math
import numpy
import rospy
from reactive_sequence import RSequence

class localizeBehaviour(pt.behaviour.Behaviour):
    def calculateConvergance(self, poseArray):
        """ :type poseArray: PoseArray"""
        maxDist = -1
        if len(poseArray.poses) <= 1001:
            referencePosition = poseArray.poses[0].position
            allowedDifference = 0.5
            for i in range(len(poseArray.poses)):
                otherPoint = poseArray.poses[i].position
                dist = math.hypot(referencePosition.y - otherPoint.y, referencePosition.x - otherPoint.x)
                # xDiff = numpy.abs(referencePosition.x - otherPoint.x)
                if dist > allowedDifference:
                    self.hasConverged = False
                    return
                elif dist > maxDist:
                    maxDist = dist
            self.hasConverged = True
        else:
            self.hasConverged = False

    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.reset()
        self.move_msg = Twist()
        self.move_msg.angular.z = 1
        self.rate = rospy.Rate(10)
        self.hasConverged = True
        self.clear_costmap_srv = rospy.ServiceProxy(rospy.get_param(rospy.get_name() + '/clear_costmaps_srv'), Empty)
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.global_particle_srv = rospy.ServiceProxy(rospy.get_param(rospy.get_name() + '/global_loc_srv'), Empty)
        self.particleService = rospy.Subscriber("/particlecloud", PoseArray, callback=self.calculateConvergance)
        super(localizeBehaviour, self).__init__("Localizer")

    def reset(self):
        rospy.loginfo("------Reset : Localizer")
        self.localizeCalled = False
        self.spunAround = False
        self.counter = 0
        self.blackboard.localizationDone = False

    def initialise(self):
        # When you are "done" and the map is dirty, reset.
        if self.blackboard.localizationDone and self.blackboard.mapIsDirty:
            self.reset()

    def update(self):
        # first, call localizeSrv
        if not self.localizeCalled:
            rospy.loginfo("Globalizing particles")
            self.global_particle_srv()
            rospy.sleep(1)
            self.hasConverged = False
            self.localizeCalled = True
        elif not self.spunAround:
            # spin around
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            self.counter += 1
            self.spunAround = self.hasConverged
        else:
            self.clear_costmap_srv()
            rospy.sleep(2)
            rospy.loginfo("Map is clean")
            self.blackboard.localizationDone = True
            self.blackboard.mapIsDirty = False
        if self.blackboard.localizationDone:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING


class isMapClean(pt.behaviour.Behaviour):
    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.previousCovariance = None
        self.covarianceHasJumped = False
        self.counter = 0
        self.robotPoseListener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        super(isMapClean, self).__init__("Map checker")

    def update(self):
        if self.blackboard.mapIsDirty:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.SUCCESS

    def pose_callback(self, robotPoseWithCovarianceStamped):
        covariance = numpy.array(robotPoseWithCovarianceStamped.pose.covariance)
        if self.previousCovariance is not None and self.blackboard.localizationDone:
            dist = numpy.linalg.norm(covariance - self.previousCovariance)
            if dist >= 0.0025:
                self.counter += 1
                if self.counter >= 3:
                    self.blackboard.mapIsDirty = True
                    rospy.loginfo("---------------------------Map Marked As Dirty--------------------------------")
            else:
                self.counter = 0

        self.previousCovariance = covariance


class retrieveCube(pt.behaviour.Behaviour):

    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        pick_cube_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        self.pick_cube_srv = rospy.ServiceProxy(pick_cube_srv_nm, SetBool)
        self.rate = rospy.Rate(10)
        self.reset()
        self.move_msg = Twist()
        self.move_msg.linear.x = -1
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(pick_cube_srv_nm, timeout=30)
        super(retrieveCube, self).__init__("Cube retriever")

    def reset(self):
        rospy.loginfo("------Reset : Retrieval")
        self.headPlacedDown = False
        self.headPlacedUp = False
        self.cubePickedUp = False
        self.movedBack = False
        self.counter = 0

    def initialise(self):
        if self.blackboard.resetPick:
            self.blackboard.resetPick = False
            self.reset()

    def update(self):
        if self.movedBack:
            return pt.common.Status.SUCCESS
        if not self.headPlacedDown:
            rospy.loginfo("Head down for cube pickup..")
            self.move_head_req = self.move_head_srv("down")
            rospy.sleep(1)
            self.headPlacedDown = True
        elif not self.cubePickedUp:
            rospy.loginfo("Picking up cube...")
            self.pick_cube_req = self.pick_cube_srv(True)
            if not self.pick_cube_req:
                rospy.loginfo("FATAL ERROR -- FAILED TO PICK UP CUBE")
                return pt.common.Status.FAILURE
            # Change goal when cube has been picked up
            self.cubePickedUp = True
            self.blackboard.cubeLocation = "Hand"

        elif not self.headPlacedUp:
            rospy.loginfo("Head up for navigation..")
            self.move_head_req = self.move_head_srv("up")
            self.rate.sleep()
            self.headPlacedUp = True
        elif not self.movedBack:
            rospy.loginfo("Moving backwards to avoid smacking into table...")
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            self.counter += 1
            if self.counter >= 5:
                self.movedBack = True
                self.blackboard.goal_position = "/place_pose_topic"

        return pt.common.Status.RUNNING


class tuckarm(pt.behaviour.Behaviour):
    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):
        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        self.blackboard = pt.blackboard.Blackboard()
        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.reset()
        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def reset(self):
        rospy.loginfo("------Reset : Arm Tucker")
        self.sent_goal = False
        self.finished = False

    def initialise(self):
        if self.blackboard.resetTuckArm:
            self.blackboard.resetTuckArm = False
            self.sent_goal = False
            self.finished = False

    def update(self):

        # already tucked the arm
        if self.finished:
            return pt.common.Status.SUCCESS

        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            rospy.loginfo("Tucking our arm...")
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class placeDownCube(pt.behaviour.Behaviour):
    def __init__(self):
        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        self.blackboard = pt.blackboard.Blackboard()
        place_cube_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_cube_Srv = rospy.ServiceProxy(place_cube_srv_nm, SetBool)
        rospy.wait_for_service(place_cube_srv_nm, timeout=30)

        # execution checker
        self.reset()

        super(placeDownCube, self).__init__("Cube placer")

    def reset(self):
        rospy.loginfo("------Reset : Placer")
        self.tried = False
        self.done = False
        self.headPlacedDown = False
        self.headPlacedUp = False

    def initialise(self):
        if self.blackboard.resetPlace:
            self.blackboard.resetPlace = False
            self.reset()

    def terminate(self, new_status):
        if self.status != pt.common.Status.FAILURE and new_status == pt.common.Status.FAILURE:
            rospy.logerr("SIMULATION FATAL ERROR: UNABLE TO MOVE ARMS. RESET SIMULATION")

    def update(self):
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS
        # try if not tried
        if not self.headPlacedDown:
            rospy.loginfo("Head down for cube placing..")
            self.move_head_req = self.move_head_srv("down")
            rospy.sleep(1)
            self.headPlacedDown = True
            return pt.common.Status.RUNNING
        elif not self.tried:

            # place up cube
            rospy.loginfo("Placing cube down...")
            rospy.sleep(1)
            self.place_cube_req = self.place_cube_Srv(True)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_cube_req.success:
            self.done = True
            rospy.loginfo("Head up for navigation..")
            self.move_head_req = self.move_head_srv("up")
            rospy.sleep(1)
            self.headPlacedUp = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_cube_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class navigateToGoal(pt.behaviour.Behaviour):
    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()

        self.navigation_result = None  # Before done, we dont know the result
        self.tried = False
        self.done = False
        self.blackboard = pt.blackboard.Blackboard()
        self.currentGoal = "/pick_pose_topic"
        self.blackboard.goal_position = self.currentGoal
        super(navigateToGoal, self).__init__("Navigator")

    def initialise(self):
        if self.currentGoal != self.blackboard.goal_position:
            self.tried = False
            self.done = False
            self.currentGoal = self.blackboard.goal_position
            # Switching from B to A

    def update(self):
        # success if done
        if self.status != pt.common.Status.INVALID and self.done:  ##Could use current location to check if at desired position.
            return self.navigation_result
        # try if not tried
        elif not self.tried:
            rospy.loginfo("Moving to goal...")
            pose = rospy.wait_for_message(self.currentGoal, PoseStamped, 7)
            goal = MoveBaseGoal()
            goal.target_pose = pose
            self.move_base_ac.send_goal(goal, done_cb=self.doneNavigating)
            self.tried = True

            return pt.common.Status.RUNNING

        # if still trying
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == pt.common.Status.FAILURE:
            # reset this node, and mark map as dirty
            rospy.loginfo("  %s [%s->%s]" % (self.name, self.status, new_status))
            # self.move_base_ac.cancel_all_goals()
            self.blackboard.mapIsDirty = True
            rospy.loginfo(
                "---------------------------Map Marked As Dirty-------------------------------- status {} , -> {}".format(
                    self.status, new_status))
            rospy.sleep(1)
            self.tried = False
            self.done = False
        elif new_status == pt.common.Status.INVALID and not self.done:
            rospy.loginfo("  %s [%s->%s]" % (self.name, self.status, new_status))
            # self.move_base_ac.cancel_all_goals()
            self.tried = False

    def doneNavigating(self, state, result):

        # result = self.move_base_ac.get_result()
        rospy.loginfo("-------------------------------Callback on navigation {}".format(state))
        # We get state == 3 when we achieve our goal position....
        if not (state == 3):  # Could be cancelled, not
            rospy.loginfo("Failed to reach desired position!")
            self.navigation_result = pt.common.Status.FAILURE
        else:
            if self.currentGoal == "/pick_pose_topic":
                rospy.loginfo("Reached goal position!  -------- A")
                self.blackboard.robotLocation = "A"
            else:
                rospy.loginfo("Reached goal position!  -------- B")
                self.blackboard.robotLocation = "B"
            self.navigation_result = pt.common.Status.SUCCESS
        self.done = True


class resetmission(pt.behaviour.Behaviour):
    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.resetService = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.move_msg = Twist()
        self.move_msg.linear.x = -5
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        super(resetmission, self).__init__("Reset mission")

    def update(self):
        rospy.loginfo("Resetting mission")
        self.blackboard.goal_position = "/pick_pose_topic"
        self.blackboard.resetTuckArm = True
        self.blackboard.resetPick = True
        self.blackboard.resetPlace = True
        self.resetCubePosition()
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.sleep(2)
        return pt.common.Status.SUCCESS

    # Resets cube position to table A
    def resetCubePosition(self):
        modelState = ModelState()
        modelState.model_name = 'aruco_cube'
        modelState.pose.position = Point(-1.130530, -6.653650, 0.86250)
        modelState.pose.orientation = Quaternion(0, 0, 0, 1)
        modelState.twist.linear = Vector3(0, 0, 0)
        modelState.twist.angular = Vector3(0, 0, 0)
        modelState.reference_frame = 'map'
        startState = SetModelStateRequest()
        startState.model_state = modelState
        self.resetService(startState)


class missionChecker(pt.behaviour.Behaviour):
    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.aruco_pose_subs = None
        self.tried = False
        super(missionChecker, self).__init__("Mission checker")

    def update(self):
        if not self.tried:
            self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)
            self.tried = True
        elif self.blackboard.cubeLocation == "B":
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def terminate(self, new_status):
        if self.status != pt.common.Status.SUCCESS and new_status == pt.common.Status.SUCCESS:
            rospy.loginfo("++++++++++++++Great success!+++++++++++++")

    def aruco_pose_cb(self, aruco_pose_msg):
        # Limitation, will not see cube teleporting away. Could we have "can see cube"? Only get update when we seee it..
        # Could have a timer counter, that increases on this function call and decreases on succesful tick.
        self.blackboard.cubeLocation = self.blackboard.robotLocation


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
        # Reset head up for gazebo simulation failures to reset robot configuration.
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        blackboard = pt.blackboard.Blackboard()
        blackboard.mapIsDirty = True
        blackboard.cubeLocation = "A"  # A, Hand, or B
        blackboard.robotLocation = None  # None, A, or B
        blackboard.resetTuckArm = False
        blackboard.resetPick = False
        blackboard.resetPlace = False

        relocateCube = RSequence(name="Cube sequence",
                                 children=[tuckarm(),
                                           localization(),
                                           navigateToGoal(),
                                           retrieveCube(),
                                           placeDownCube(),
                                           resetmission()
                                           ])
        tree = pt.composites.Selector(name="Main sequence",
                                      children=[missionChecker(), relocateCube]
                                      )
        super(BehaviourTree, self).__init__(tree)
        # ensure head is up before starting, otherwise need to reset gazebo every time.
        move_head_srv("up")
        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown():
            self.tick_tock(1)


def localization():
    return pt.composites.Selector(
        name="Checked localization",
        children=[isMapClean(), localizeBehaviour()]
    )


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()