#!/usr/bin/env python3

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist,PoseStamped,Pose

from nav_msgs.msg import Path, OccupancyGrid
import tf_transformations

from tf2_msgs.msg import TFMessage

import numpy as np
from geometry_msgs.msg import TransformStamped,PoseStamped
import math

# ros multi-Threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

import threading

import cv2

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from threading import RLock
from rocap_ros.action import Move, FollowPath

import time

class Grid:

    def __init__(self,occupencyGrid:OccupancyGrid):
        self.width = occupencyGrid.info.width
        self.heigth = occupencyGrid.info.height
        self.res = occupencyGrid.info.resolution
        self.origine = occupencyGrid.info.origin.position

        # np_data = np.array([self.color_converter(e) for e in occupencyGrid.data])
        np_data = np.array([occupencyGrid.data])
        self.grid = np.reshape(np_data, (self.heigth,self.width))
        self.occupencyGrid = occupencyGrid
    
    def getResolution(self):
        return self.res

    def getCellAtPoint(self,x:float,y:float):
        targetCellx = math.ceil((x-self.origine.x)/self.res)
        targetCelly = math.ceil((y-self.origine.y)/self.res)
        return self.grid[targetCelly][targetCellx]
    
    def isObstacleAtPoint(self,x:float,y:float):
        if self.getCellAtPoint(x,y) > 0.99:
            return True
        else:
            return False
    
    def color_converter(self,e):
        if(e == -1):
            return 255
        elif e < 99:
            return 255
        else:
            return 0 
        
    def saveGrid(self, path):
        np_data = np.array([self.color_converter(e) for e in self.occupencyGrid.data])
        reshaped = np.flipud(np.reshape(np_data, (self.occupencyGrid.info.height, self.occupencyGrid.info.width)))
        cv2.imwrite(path, reshaped)


class localPlanner(Node):

    def __init__(self):
        
        super().__init__('LocalPlanner',parameter_overrides=[rclpy.Parameter('use_sim_time',
                                                                             type_=rclpy.Parameter.Type.BOOL,
                                                                             value=True)])

        self.declare_all_parameters()
        self.get_all_parameters()
        
        self.pathGrid = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.update_grid_cb,
            10)
        
        self.grid = None
        
        self.moveSubcriberCb = MutuallyExclusiveCallbackGroup()

        self.updateGoal = self.create_subscription(
            PoseStamped,
            'localPlanner/movePoint',
            self.update_target_move,
            1,callback_group=self.moveSubcriberCb)

        self.updateGoal = self.create_subscription(
            PoseStamped,
            'localPlanner/planGoal',
            self.update_target_planning,
            1,callback_group=self.moveSubcriberCb)
                
        self.pathPlanningMutex = RLock()
        self.pathPlanningRunning = False
        self.pathPlanActionFuture = Future()
        self.pathPlanActionResultFuture = Future()
        
        self.path_plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose',callback_group=MutuallyExclusiveCallbackGroup())
        self.action_client_move = ActionClient(self, 
                                           Move, 
                                           '/move_rocap',
                                           callback_group=MutuallyExclusiveCallbackGroup())
        
        self.action_client_follow_path = ActionClient(self, 
                                           FollowPath, 
                                           '/follow_path',
                                           callback_group=MutuallyExclusiveCallbackGroup())
        
        self.gridMutex = threading.RLock()

    def declare_all_parameters(self):
        self.declare_parameter('z_plane', 1.5)

    def get_all_parameters(self):
        self.z_plane = self.get_parameter('z_plane').get_parameter_value().double_value

    def update_target_move(self,msg:PoseStamped):
        self.get_logger().info("uptate target move")
        self.send_move_async(msg)
       
    def update_target_planning(self,msg:PoseStamped):
        self.get_logger().info("uptate target planning")
        self.move_using_planning(msg)
        

    def move_using_planning(self, msg):
        
        planning = True
        while planning :
            planning = False # set to True if replan is nessesery
            path = self.get_path(msg)

            if  self.is_valid_path(path):
                goalAsync:Future = self.send_followPath_async(path)
                goal_handle:ClientGoalHandle = self.wait_for_goal_handle(goalAsync)
 
                if goal_handle.accepted:
                    self.get_logger().info("follow path goal accepted watching traversability change")
                    follow_path_action_result:Future = goal_handle.get_result_async()

                    while not follow_path_action_result.done():
                        self.gridMutex.acquire()
                        if self.isObstaclesOnPath(self.grid,path):
                            goal_handle.cancel_goal()
                            print("obstacle on path detected replanning")
                            planning = True
                        self.gridMutex.release()
                        time.sleep(0.1)

                if not planning: 
                    resultRequest = follow_path_action_result.result()
                    if resultRequest.status == GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().info("follow path succed")
                    else:
                        self.get_logger().info("follow path, status:={0}".format(resultRequest.status))
                        goalAsync.result()

    def wait_for_goal_handle(self, goalAsync) -> ClientGoalHandle:
        while not goalAsync.done():
            time.sleep(0.01)

        return goalAsync.result()

    def is_valid_path(self, path):
        return path != None and len(path.poses) > 0 

        
    def send_move_async(self, targetPosition:PoseStamped):
            goal_msg = Move.Goal()
            goal_msg.target = targetPosition
            goal_msg.target.pose.position.z = self.z_plane
            goal_msg.target.header.stamp = self.get_clock().now().to_msg()

            self.action_client_move.wait_for_server()
            self.get_logger().info("sending move cmd")
            moveActionFuture = self.action_client_move.send_goal_async(goal_msg)
            moveActionFuture.add_done_callback(self.send_move_complete)

    def send_followPath_async(self, path:Path) -> Future:
            goal_msg = FollowPath.Goal()
            goal_msg.target = path
            goal_msg.target.header.stamp = self.get_clock().now().to_msg() ### TODO remove line

            self.action_client_move.wait_for_server()
            self.get_logger().info("send to follow path")
            return self.action_client_follow_path.send_goal_async(goal_msg)

    def send_followPath_complete(self, future:Future):
        goal_handle:ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("follow path goal accepted")
            moveActionResultFuture:Future = goal_handle.get_result_async()
            moveActionResultFuture.add_done_callback(self.get_followPath_result)

    def send_move_complete(self, future:Future):
        goal_handle:ClientGoalHandle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("move goal accepted")
            moveActionResultFuture:Future = goal_handle.get_result_async()
            moveActionResultFuture.add_done_callback(self.get_move_result)

    def get_move_result(self, future:Future):
        resultRequest = future.result()
        if resultRequest.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("move goal succed")
        else:
            self.get_logger().info("goal fail, status:={0}".format(resultRequest.status))

    def get_followPath_result(self, future:Future):
        resultRequest = future.result()
        if resultRequest.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("follow path action succed")
        else:
            self.get_logger().info("follow path action fail, status:={0}".format(resultRequest.status))

    def get_path(self, targetPosition:PoseStamped):
        with self.pathPlanningMutex:
            self.pathPlanningRunning = True
            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = targetPosition
            goal_msg.goal.header.stamp = self.get_clock().now().to_msg()

            self.path_plan_client.wait_for_server()
            self.get_logger().info("computing path cmd")
            pathRequest = self.path_plan_client.send_goal(goal_msg)

            if pathRequest.status == GoalStatus.STATUS_SUCCEEDED:
                
                if self.grid != None:
                    self.gridMutex.acquire()
                    path = self.compute_smooted_path(pathRequest.result.path,self.grid)
                    
                    if self.isObstaclesOnPath(self.grid,path):
                        self.get_logger().info("computing path fail")
                        self.gridMutex.release()
                        return None
                    
                    self.gridMutex.release()
                    
                    self.get_logger().info("computing path succed")
                    return path
            else:
                self.get_logger().info("get path fail, status:={0}".format(pathRequest.status))
            
            return None

    def update_grid_cb(self,msg:OccupancyGrid):
        self.gridMutex.acquire()
        self.grid = Grid(msg)
        self.gridMutex.release()

    def getAngle(self,vectO,vectN):
        dotProd = vectO.dot(vectN)
        norm = (np.linalg.norm(vectO)*np.linalg.norm(vectN))
        return math.acos(max(-1,min(dotProd/norm,1.0)))

    def getVec(self, point1, point2):
        return np.array([point2.x-point1.x,
                  point2.y-point1.y])

    def isObstaclesOnVertex(self,grid:Grid,origin,dest):
        dirVect = self.getVec(origin,dest)
        dist = np.linalg.norm(dirVect)
        dirVect = dirVect/dist
        dirX = dirVect[0]
        dirY = dirVect[1]

        decimation = math.ceil(dist/(grid.getResolution()/2))
        for ctr in range(0,decimation):
            px = origin.x+dirX*(dist/decimation)*ctr
            py = origin.y+dirY*(dist/decimation)*ctr
            if grid.isObstacleAtPoint(px,py):
                return True
        
        return False
    
    def isObstaclesOnPath(self,grid:Grid, path:Path):
        pathPose = path.poses
        for ctr in range(len(pathPose)-1):
            if self.isObstaclesOnVertex(grid,pathPose[ctr].pose.position,pathPose[ctr+1].pose.position):
                return True        
        return False

    def compute_smooted_path(self,path:Path, grid:Grid) -> Path:
        origine = path.poses[0].pose.position
        next = path.poses[1].pose.position

        approxVec = self.getVec(origine,next)
        filterPath = Path()
        filterPath.header = path.header
        
        fistPose = PoseStamped()
        fistPose.pose.position.x = origine.x
        fistPose.pose.position.y = origine.y
        fistPose.pose.position.z = self.z_plane
        filterPath.poses.append(fistPose)

        ctr = 2
        lastPoint = next
        while ctr < len(path.poses):

            newPoint = path.poses[ctr].pose.position
            vectN = self.getVec(origine,newPoint)


            angle = self.getAngle(approxVec,vectN)
            if angle < 0.0872665*2 and not self.isObstaclesOnVertex(grid,origine,newPoint):
            # if angle < 0.0872665*3:
                ctr+=1
                lastPoint = newPoint
            else:
                poseStamp = PoseStamped()
                poseStamp.pose.position.x = lastPoint.x
                poseStamp.pose.position.y = lastPoint.y
                poseStamp.pose.position.z = self.z_plane
                filterPath.poses.append(poseStamp)

                origine = lastPoint
                lastPoint = newPoint
                approxVec = self.getVec(origine,lastPoint)
                ctr+=1
            
        lastPose = PoseStamped()
        lastPose.pose.position.x = newPoint.x
        lastPose.pose.position.y = newPoint.y
        lastPose.pose.position.z = self.z_plane
        filterPath.poses.append(lastPose)
        
        print(len(path.poses),len(filterPath.poses))

        return filterPath
        
        
        
def main(args=None):
    rclpy.init(args=args)


    executor = MultiThreadedExecutor()
    minimal_subscriber = localPlanner()
    executor.add_node(minimal_subscriber)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()