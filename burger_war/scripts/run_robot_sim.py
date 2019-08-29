#!/usr/bin/env python

import rospy
import math
import urllib
import json
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class MyBot():
    def __init__(self, robot_name):
        self.name = robot_name
        self.__listener = tf.TransformListener()
        self.__ac = actionlib.SimpleActionClient(
            "/{}/move_base".format(self.name), MoveBaseAction)
        self.__ac.wait_for_server()

        # targtet position
        self.target_pos_dict = {
            "Omelette_N": [0.53, 0.855, -90],
            "Omelette_S": [0.53, 0.205, 90],
            "Tomato_N": [-0.53, 0.855, -90],
            "Tomato_S": [-0.53, 0.205, 90],
            "Pudding_N": [-0.53, -0.205, -90],
            "Pudding_S": [-0.53, -0.855, 90],
            "OctopusWiener_N": [0.53, -0.205, -90],
            "OctopusWiener_S": [0.53, -0.855, 90],
            "FriedShrimp_N": [0, 0.425, -90],
            "FriedShrimp_E": [0.425, 0, 180],
            "FriedShrimp_S": [0, -0.425, 90],
            "FriedShrimp_W": [-0.425, 0, 0],
            "BL_B": [None, None, None],
            "BL_L": [None, None, None],
            "BL_R": [None, None, None],
            "RE_B": [None, None, None],
            "RE_L": [None, None, None],
            "RE_R": [None, None, None]
        }
        self.__pre_score = {}
        self.updateScore()

        #yamaguchi add
        self.enemyPose_sub = rospy.Subscriber("/{}/enemyPose".format(self.name),PoseStamped,self.enemyPoseUpdate)
    
    def enemyPoseUpdate(self, enemyPose):
        #print(enemyPose)
        #print(enemyPose.pose.position.x)
        #print(enemyPose.pose.position.y)
        enemyPose.header.stamp = self.__listener.getLatestCommonTime("/{}/odom".format(self.name), "/{}/base_scan".format(self.name))
        p = self.__listener.transformPose("{}/odom".format(self.name), enemyPose)
        #print(p.pose.position.x)
        #print(p.pose.position.y)
        if enemyPose.pose.position.x == 0 and enemyPose.pose.position.y == 0:
            if self.name == "red_bot":
                self.target_pos_dict["BL_B"] = [None, None, None]
            elif self.name == "blue_bot":
                self.target_pos_dict["RE_B"] = [None, None, None]
        else:
            #print([p.pose.position.x, p.pose.position.y, 2 * math.asin(p.pose.orientation.z)])
            if self.name == "red_bot":
                self.target_pos_dict["BL_B"] = [p.pose.position.x, p.pose.position.y, 2 * math.asin(p.pose.orientation.z)/math.pi*180]
            elif self.name == "blue_bot":
                self.target_pos_dict["RE_B"] = [p.pose.position.x, p.pose.position.y, 2 * math.asin(p.pose.orientation.z)/math.pi*180]
            #print("############# enemy update!!!! #######################")
        #print(self.target_pos_dict)

    def getRadFromDeg(self, deg):
        return deg/180.0*math.pi

    def generatePose(self, x, y, th):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/{}/odom".format(self.name)
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(
            self.getRadFromDeg(th)/2)
        goal.target_pose.pose.orientation.w = math.cos(
            self.getRadFromDeg(th)/2)
        return goal

    def sendGoal(self, x, y, th):
        self.__ac.send_goal(self.generatePose(x, y, th))

    def __jsonConversion(self, jsonStr):
        data = json.loads(jsonStr.read().decode("utf-8"))
        return data

    def getScoreState(self):
        url = "http://localhost:5000/warState"
        state = self.__jsonConversion(urllib.urlopen(url))
        return state["targets"]

    def updateScore(self):
        new_score = self.getScoreState()
        if self.__pre_score != new_score:
            self.__pre_score = new_score
            self.unscored_dict = {}
            self.scored_dict = {}
            self.enemy_scored_dict = {}
            for target in self.__pre_score:
                if target["player"] == "n":
                    #print(target["name"])
                    self.unscored_dict[target["name"]
                                       ] = self.target_pos_dict[target["name"]]
                if target["player"] == "r":
                    if self.name == "red_bot":
                        self.scored_dict[target["name"]
                                         ] = self.target_pos_dict[target["name"]]
                    else:
                        self.enemy_scored_dict[target["name"]
                                               ] = self.target_pos_dict[target["name"]]
                if target["player"] == "b":
                    if self.name == "blue_bot":
                        self.scored_dict[target["name"]
                                         ] = self.target_pos_dict[target["name"]]
                    else:
                        self.enemy_scored_dict[target["name"]
                                               ] = self.target_pos_dict[target["name"]]

    def getPosition(self):
        now = rospy.Time.now()
        self.__listener.waitForTransform(
            "{}/odom".format(self.name), "{}/base_link".format(self.name), now, rospy.Duration(1.0))
        position, quaternion = self.__listener.lookupTransform(
            "{}/odom".format(self.name), "{}/base_link".format(self.name), now)
        return position, quaternion


if __name__ == '__main__':
    rospy.init_node("simple_navigation_goals")
    my_bot = MyBot(rospy.get_namespace().replace("/", ""))

    current_goal = []
    # main loop
    while not rospy.is_shutdown():
        # check current score
        my_bot.updateScore()

        # retarget
        position, quaternion = my_bot.getPosition()

        new_goal = []

        targets_dict = my_bot.unscored_dict
        targets_dict.update(my_bot.enemy_scored_dict)
        #print(targets_dict)
        for target in targets_dict.values():
            if target[0] == None:
                continue
            elif len(new_goal) == 0:
                new_goal = target
            elif ((position[0]-target[0])**2+(position[1]-target[1])**2) < ((position[0]-new_goal[0])**2+(position[1]-new_goal[1])**2):
                new_goal = target
        
        if my_bot.name == "red_bot":
            #print(my_bot.target_pos_dict["BL_B"])
            if my_bot.target_pos_dict["BL_B"] != [None,None,None]:
                new_goal = my_bot.target_pos_dict["BL_B"]
                #print("############# enemy goal set!!!! #######################")
        elif my_bot.name == "blue_bot":
            #print(my_bot.target_pos_dict["RE_B"])
            if my_bot.target_pos_dict["RE_B"] != [None,None,None]:
                new_goal = my_bot.target_pos_dict["RE_B"]
                #print("############# enemy goal set!!!! #######################")

        #print(new_goal)
        if new_goal != current_goal:
            #print("new target")
            current_goal = new_goal
            my_bot.sendGoal(current_goal[0], current_goal[1], current_goal[2])
        rospy.sleep(0.5)
