#!/usr/bin/env python
import py_trees
import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from geometry_msgs.msg import PoseArray, Pose, Point
from reactive_sequence import RSequence
import math

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
        # Reset head up for gazebo simulation failures to reset robot configuration.
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        blackboard = py_trees.blackboard.Blackboard()
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
