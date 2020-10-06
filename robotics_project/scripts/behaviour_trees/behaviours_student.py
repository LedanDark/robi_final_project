# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.
import numpy
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import SetBool, Empty


class counter(pt.behaviour.Behaviour):
    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):
        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):
        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):
    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):
        rospy.loginfo("Initialising go behaviour.")

        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):
        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING


class localizeBehaviour(pt.behaviour.Behaviour):
    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.reset()
        self.move_msg = Twist()
        self.move_msg.angular.z = 1
        self.rate = rospy.Rate(10)
        self.clear_costmap_srv = rospy.ServiceProxy(rospy.get_param(rospy.get_name() + '/clear_costmaps_srv'), Empty)
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.localize_service = rospy.ServiceProxy(rospy.get_param(rospy.get_name() + '/global_loc_srv'), Empty)
        super(localizeBehaviour, self).__init__("Localize Behaviour")

    def reset(self):
        rospy.loginfo("Localizer is resetting")
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
            rospy.loginfo("Calling localize service")
            # TODO : Check if this is too fast
            self.localize_service()
            self.rate.sleep()
            self.localizeCalled = True
        elif not self.spunAround:
            # spin around
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            self.counter += 1
            self.spunAround = self.counter >= 60
        else:
            rospy.loginfo("Calling clear costmap service")
            self.clear_costmap_srv()
            self.rate.sleep()
            self.blackboard.localizationDone = True
            self.blackboard.mapIsDirty = False
        if self.blackboard.localizationDone:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING


class isMapDirty(pt.behaviour.Behaviour):
    def __init__(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.previousCovariance = None
        self.covarianceHasJumped = False
        self.counter = 0
        self.subramanian = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        super(isMapDirty, self).__init__("Is Map clean?")

    def update(self):
        if self.blackboard.mapIsDirty:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.SUCCESS

    def pose_callback(self, poseWithCovarianceStamped):
        covariance = numpy.array(poseWithCovarianceStamped.pose.covariance)
        if self.previousCovariance is not None and self.blackboard.localizationDone:
            dist = numpy.linalg.norm(covariance - self.previousCovariance)
            # rospy.loginfo("Distance == {}".format(dist))
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
        self.reset()
        super(retrieveCube, self).__init__("Retrieve cube")

    def reset(self):
        self.headPlacedDown = False
        self.headPlacedUp = False
        self.cubePickedUp = False
        self.movedBack = False
        self.counter = 0
        pass

    def update(self):
        if self.movedBack:
            return pt.common.Status.SUCCESS
        if not self.headPlacedDown:
            rospy.loginfo("%s: Head down for cube pickup..")
            self.move_head_req = self.move_head_srv("down")
            self.rate.sleep()
            self.headPlacedDown = True
        elif not self.cubePickedUp:
            rospy.loginfo("%s: Picking up cube...")
            self.pick_cube_req = self.pick_cube_srv(True)
            if not self.pick_cube_req:
                rospy.loginfo("%s: FATAL ERROR -- FAILED TO PICK UP CUBE")
                return pt.common.Status.FAILURE
            self.cubePickedUp = True
        elif not self.headPlacedUp:
            rospy.loginfo("%s: Head up for navigation..")
            self.move_head_req = self.move_head_srv("up")
            self.rate.sleep()
            self.headPlacedUp = True
        elif not self.movedBack:
            rospy.loginfo("Moving backwards to avoid smacking into table...")
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            self.counter += 1
            self.movedBack = self.counter >= 3
            pass
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

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished:
            return pt.common.Status.SUCCESS

        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
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


class pickUpCube(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("Initialising pick up cube behaviour.")
        # server
        pick_cube_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_cube_srv = rospy.ServiceProxy(pick_cube_srv_nm, SetBool)
        rospy.wait_for_service(pick_cube_srv_nm, timeout=30)

        # execution checker
        self.tried = False
        self.done = False

        super(pickUpCube, self).__init__("Pick up cube!")

    def update(self):
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS
        # try if not tried
        elif not self.tried:

            # Pick up cube
            rospy.loginfo("%s: Picking up cube...")
            self.pick_cube_req = self.pick_cube_srv(True)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_cube_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_cube_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class placeDownCube(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("Initialising place down cube behaviour.")
        # server
        rospy.loginfo("%s: Placing cube down...")
        place_cube_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_cube_Srv = rospy.ServiceProxy(place_cube_srv_nm, SetBool)
        rospy.wait_for_service(place_cube_srv_nm, timeout=30)

        # execution checker
        self.tried = False
        self.done = False

        super(placeDownCube, self).__init__("Place down cube!")

    def update(self):
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS
        # try if not tried
        elif not self.tried:

            # Pick up cube
            rospy.loginfo("%s: Placing cube down...")
            rospy.sleep(1)
            self.pick_cube_req = self.place_cube_Srv(True)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_cube_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_cube_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class movehead(pt.behaviour.Behaviour):
    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            rospy.sleep(1)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class moveTo(pt.behaviour.Behaviour):
    def __init__(self, given_topic):
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()

        self.navigation_result = None  # Before done, we dont know the result
        self.tried = False
        self.done = False
        self.blackboard = pt.blackboard.Blackboard()
        self.pose_topic = given_topic
        super(moveTo, self).__init__("Moving to " + given_topic)

    def update(self):
        # success if done
        if self.done:  ##Could use current location to check if at desired position.
            return self.navigation_result
        # try if not tried
        elif not self.tried:
            pose = rospy.wait_for_message(self.pose_topic, PoseStamped, 7)
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
            self.blackboard.mapIsDirty = True
            self.tried = False
            self.done = False
            rospy.loginfo("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    def doneNavigating(self, state, result):

        # result = self.move_base_ac.get_result()
        rospy.loginfo("-------------------------------Callback on navigation")
        # We get state == 3 when we achieve our goal position....
        if not (state == 3):
            self.move_base_ac.cancel_goal()
            rospy.loginfo("Failed to reach desired position!")
            self.navigation_result = pt.common.Status.FAILURE
        else:
            rospy.loginfo("Reached goal position!")
            self.navigation_result = pt.common.Status.SUCCESS
        self.done = True
