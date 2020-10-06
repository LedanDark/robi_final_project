#!/usr/bin/env python
import py_trees
import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
        # Reset head up for gazebo simulation failures to reset robot configuration.
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        blackboard = py_trees.blackboard.Blackboard()
        blackboard.mapIsDirty = True
        blackboard.headState = None
        # lower head
        headDown = movehead("down")

        # localization, headUp and spin

        # move backwards
        moveBackwards = pt.composites.Selector(
            name="Go backwards",
            children=[counter(2, "Away from table?"), go("Fall back from chair", -1, 0)]
        )
        # become the tree
        tree = RSequence(name="Main sequence",
                         children=[tuckarm(),
                                   localization(),
                                   moveTo("/pick_pose_topic"),
                                   retrieveCube(),
                                   moveTo("/place_pose_topic"),
                                   placeDownCube()])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        move_head_srv("up") #ensure head is up before starting, otherwise need to reset gazebo every time.
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown():
            self.tick_tock(1)


def localization():
    return pt.composites.Selector(
        name="Checked localization",
        children=[isMapDirty(), localizeBehaviour()]
    )


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
