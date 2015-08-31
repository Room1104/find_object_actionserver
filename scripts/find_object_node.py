#!/usr/bin/env python

import rospy

from find_object_actionserver.msg import test_actionAction
from find_object_actionserver.msg import test_actionResult
from datetime import *
from std_srvs.srv import Empty, EmptyResponse
from strands_executive_msgs import task_utils
from strands_executive_msgs.abstract_task_server import AbstractTaskServer


#my changes starts here
import uuid
from strands_executive_msgs import Task
from strands_executive.msgs.srv import AddTask, DemandTask,SetExecutionStats
import std_srvs.srv
import mary_tts.msg
import sensor_msgs.msg
import flir_pantilt_d46.msg
from sensor_msgs.msg import JointState


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

    def say(text):
        # code copied from QtGUI
        c = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)
        if(not c.wait_for_server(timeout=rospy.Duration(10))):
            print "Failed to connect to server"
            return std_srvs.srv.TriggerResponse(False,"failed to connect to server");
        goal = mary_tts.msg.maryttsGoal(text)
        c.send_goal(goal)
        if(not c.wait_for_result(timeout=rospy.Duration(10))):
            print "Failed to get result"
            return std_srvs.srv.TriggerResponse(False,"failed to get result");
        return std_srvs.srv.TriggerResponse(True,text);

    def execute(self, goal):


        # TODO: CHECK IF IT IS COMPILING


        now = rospy.get_rostime()
        order = goal.order
        rospy.loginfo("order is: %s" 
                          % order)

        result = test_actionResult()
        result.sequence.append(5)

        say("I'm trying to find the coffee!")

        # start doing something
        coffee_found = False
        list_of_waypoints = ['WayPoint1','WayPoint2','WayPoint3','WayPoint4','WayPoint5']
        for i in list_of_waypoints:
                
            self.say("I'm going to " + i)
            # ACTION : GO TO WAYPONT

            # Move to the next location
            task = Task()

            task.action = '/wait_action'

            max_wait_minutes = 160 * 160
            task.max_duration = rospy.Duration(max_wait_minutes)

            task.start_node_id = i
            task.end_node_id = i

            task_utils.add_time_argument(task, rospy.Time())
            task_utils.add_duration_argument(task, rospy.Duration(10))

            set_execution_status = get_execution_status_service()
            set_execution_status(True) 

            task.start_after = rospy.get_rostime() + rospy.Duration(10)
            task.end_before = task.start_after + rospy.Duration(task.max_duration.to_sec() * 3)
            
            # Execute task
            demand_task = get_demand_task_service()
            demand_task(task)


            # Change the orientation of the camera
            tilt_PTU(0,40)

            # obtaining from the topic a point cloud
            msg = rospy.wait_for_message("/head_xtion/depth_registered/points", sensor_msgs.msg.PointCloud2)

            # calling the object recognition service 
            object_recog = rospy.ServiceProxy('/recognition_service/sv_recognition', recognition_srv_definitions.srv.recognize)
            object_recog.wait_for_service()
            requestCoffee = recognition_srv_definitions.srv.recognizeRequest()
            requestCoffee.cloud = msg
            results = object_recog(requestCoffee)
            results.ids
            print results.ids
            if "nescafe" in results.ids or "teabox" in results.ids :
                coffee_found = True 
                #do stuff
                #coffee_found = PSEUDOCODE_WHERE_I_SHOULD_CALL_THE_SERVICE
            else
                say("coffee not found at location "+ i)

            if coffee_found :
                coffee_place = i
                say("The coffee is at" + i)
                break
            tilt_PTU(0,0)


        if coffee_found    
            self.say("I found coffee i'm happy")
            #twit that the coffee was found
            # be happy
        else 
            #twit that someone stole the coffee
            # be sad
            self.say("Someone stole the coffee")
            # end doing something

        self.server.set_succeeded(result)

    def get_service(service_name, service_type):    
        rospy.loginfo('Waiting for %s service...' % service_name)
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")        
        return rospy.ServiceProxy(service_name, service_type)

    def get_execution_status_service():
        return get_service('/task_executor/set_execution_status', SetExecutionStatus)

    def get_add_tasks_service():
        return get_service('/task_executor/add_tasks', AddTasks)

    def get_demand_task_service():
        return get_service('/task_executor/demand_task', DemandTask)

    
    def tilt_PTU(pan,tilt)
        # PTU client
        rospy.loginfo("Creating PTU client")
        self.ptuClient = actionlib.SimpleActionClient(
            'SetPTUState',
            flir_pantilt_d46.msg.PtuGotoAction
        )
        self.ptuClient.wait_for_server()
        rospy.loginfo("...done")

        goal = flir_pantilt_d46.msg.PtuGotoGoal()
        goal.pan = pan
        goal.tilt = tilt
        goal.pan_vel = 60
        goal.tilt_vel = 60
        self.ptuClient.send_goal(goal)
    


if __name__ == '__main__':

    rospy.init_node("test_action")

    interruptible = rospy.get_param("~interruptible", True)
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown() and rospy.get_rostime().secs == 0:
        pass

    test = TestServer(interruptible=interruptible, name=rospy.get_name())

    rospy.spin()
