#!/usr/bin/env python

import rospy
from find_object_actionserver.msg import test_actionAction
from find_object_actionserver.msg import test_actionResult
from datetime import *
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs import task_utils
from strands_executive_msgs.abstract_task_server import AbstractTaskServer


class TestServer(AbstractTaskServer):
    def __init__(self, interruptible=True, name='find_object'):
        super(TestServer, self).__init__(name, action_type=test_actionAction,
                                         interruptible=interruptible)
        self.maximise_duration_delta = rospy.Duration(rospy.get_param('~maximise_duration_delta', 0))

    def create(self, req):
        t = super(TestServer, self).create(req)
        task_utils.add_time_argument(t, rospy.Time())
        if self.maximise_duration_delta > rospy.Duration(0):
            d = (t.end_before - t.start_after) - self.maximise_duration_delta
            task_utils.add_duration_argument(t, d)
            t.max_duration = d
        else:
            task_utils.add_duration_argument(t, t.max_duration)
        return t

    def execute(self, goal):
#        end_wait_srv = rospy.Service('/wait_action/end_wait',
#                                     Empty, self.end_wait)
        now = rospy.get_rostime()
        order = goal.order
        rospy.loginfo("order is: %s" 
                          % order)

        result = test_actionResult()
        result.sequence.append(5)

        self.server.set_succeeded(result)


if __name__ == '__main__':

    rospy.init_node("test_action")

    interruptible = rospy.get_param("~interruptible", True)
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    test = TestServer(interruptible=interruptible, name=rospy.get_name())

    rospy.spin()
