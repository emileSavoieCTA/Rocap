#!/usr/bin/env python3


from rocap_rest_api_client.models import Vec3
from rocap_rest_api_client.types import Response

from rocap_rest_api_client.api.displacement import linear_move
import rocap_rest_api_client.api.displacement.stop as stopAPI
from rocap_rest_api_client.api.brakes import disengage_robot_brakes, engage_the_robot_brakes
from rocap_rest_api_client.models.command_ack import CommandAck
from rocap_rest_api_client.api.brakes import disengage_robot_brakes

from rocap_rest_api_client.api.state import state_query
from rocap_rest_api_client.models.state_query import StateQuery
from rocap_rest_api_client.models.state_response import StateResponse
from rocap_rest_api_client.models.linear_move_request import LinearMoveRequest
from rocap_rest_api_client.models.linear_move_response import LinearMoveResponse
from rocap_rest_api_client.models.generic_error import GenericError

from rocap_rest_api_client.types import UNSET

from rocap_rest_api_client.api.access_token import request
from rocap_rest_api_client.models.api_v0_access_key import ApiV0AccessKey
from rocap_rest_api_client.models.rocap_control_state import RocapControlState

from rocap_rest_api_client import Client

from http import HTTPStatus

import threading

# general import
from abc import ABC, abstractmethod
import json
import math
import time
import numpy as np


# ros import
import rclpy
import rclpy.clock
from rclpy.node import Node

# ros multi-Threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.task import Future

# Ros2 Action
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rocap_ros.action import Move, FollowPath
from action_msgs.msg import GoalStatus

# Ros msg
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from nav_msgs.msg import Odometry, Path
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import Float64MultiArray

# tf import
import tf_transformations
import tf2_geometry_msgs.tf2_geometry_msgs  # if remove break tf_transform
from tf2_ros.buffer import Buffer
from tf2_msgs.msg import TFMessage
from transformlistener import TransformListener
import copy


class API(ABC):

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def set_position_goal(self, x, y, z, roll, pitch, yaw, speed) -> bool:
        pass

    @abstractmethod
    def stop(self) -> bool:
        pass

    @abstractmethod
    def disengage_brake(self) -> bool:
        pass

    @abstractmethod
    def engage_brake(self) -> bool:
        pass

    @abstractmethod
    def get_state(self) -> StateResponse:
        pass


class RocapAPIMock(API):

    def __init__(self, node: Node):
        super().__init__()
        self.node = node

        self.declare_parameters()
        self.get_all_parameters()

        self.mockCb = MutuallyExclusiveCallbackGroup()

        self.timer_period = 0.025  # seconds
        self.timer = self.node.create_timer(
            self.timer_period, self.update_state, MutuallyExclusiveCallbackGroup())

        self.velocity_goal_pub = self.node.create_publisher(Float64MultiArray,
                                                            '/velocity_controller/commands',
                                                            10,
                                                            callback_group=self.mockCb)

        self.pose = Pose()

        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=3.0))
        self.tf_listener = TransformListener(
            self.tf_buffer, self.node, tf_topic='/sim/tf', tf_static_topic='/sim/tf_static')

        self.state = RocapControlState.ON_BRAKES

        self.init = False
        self.goal = Pose()

    def declare_parameters(self):
        self.node.declare_parameter('sim_odom_topic', '/ground_truth/odom')

    def get_all_parameters(self):
        self.odomTopic = self.node.get_parameter(
            'sim_odom_topic').get_parameter_value().string_value

    def getCurrentPose(self):
        targetPose = PoseStamped()
        targetPose.pose.position.x = 0.0
        targetPose.pose.position.y = 0.0
        targetPose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0,
                                                     0.0,
                                                     0.0)

        targetPose.pose.orientation.x = q[0]
        targetPose.pose.orientation.y = q[1]
        targetPose.pose.orientation.z = q[2]
        targetPose.pose.orientation.w = q[3]

        targetPose.header.frame_id = 'base_link'
        targetPose.header.stamp.sec = 0
        targetPose.header.stamp.nanosec = 0
        poseInBase = self.tf_buffer.transform(
            targetPose, 'base_D_link', timeout=rclpy.duration.Duration(seconds=1.0))
        return poseInBase.pose

    def update_state(self):

        try:
            self.pose = self.getCurrentPose()
            if not self.init:
                self.goal = self.pose
                self.init = True

            self.update_pos_control(self.goal.position.x,
                                    self.goal.position.y,
                                    self.goal.position.z, 0, 0, 0, 0.25)

            if self.state == RocapControlState.MOVING:
                if abs(self.pose.position.x - self.goal.position.x) < 0.05:
                    if abs(self.pose.position.y - self.goal.position.y) < 0.05:
                        if abs(self.pose.position.z - self.goal.position.z) < 0.05:
                            self.state = RocapControlState.IDLE
        except:
            self.send_vel_cmd(0, 0, 0, 0, 0, 0)

    def get_state(self) -> StateResponse:
        stateResp = StateResponse()
        stateResp.state = self.state
        stateResp.position = Vec3(self.pose.position.x,
                                  self.pose.position.y,
                                  self.pose.position.z)

        rpy = tf_transformations.euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
        ])

        stateResp.orientation = Vec3(rpy[0],
                                     rpy[1],
                                     rpy[2])

        return stateResp

    def set_position_goal(self, x, y, z, roll, pitch, yaw, speed) -> bool:
        if self.state == RocapControlState.IDLE:

            self.state = RocapControlState.MOVING
            self.update_pos_control(x, y, z, roll, pitch, yaw, speed)
            self.set_target(x, y, z)
            return True
        return False

    def set_target(self, x, y, z):
        self.goal = Pose()
        self.goal.position.x = x
        self.goal.position.y = y
        self.goal.position.z = z

    def send_pos_cmd(self, x, y, z, roll, pitch, yaw):
        targetPose = PoseStamped()
        targetPose.pose.position.x = x
        targetPose.pose.position.y = y
        targetPose.pose.position.z = z

        q = tf_transformations.quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))

        targetPose.pose.orientation.x = q[0]
        targetPose.pose.orientation.y = q[1]
        targetPose.pose.orientation.z = q[2]
        targetPose.pose.orientation.w = q[3]

        targetPose.header.frame_id = 'base_D_link'
        targetPose.header.stamp = rclpy.clock.Clock().now().to_msg()
        poseInWorld = self.tf_buffer.transform(
            targetPose, 'world', timeout=rclpy.duration.Duration(seconds=1.0))

        rpy = tf_transformations.euler_from_quaternion([
            poseInWorld.pose.orientation.x,
            poseInWorld.pose.orientation.y,
            poseInWorld.pose.orientation.z,
            poseInWorld.pose.orientation.w,
        ])

        posMsg = Float64MultiArray()
        posMsg.data.append(poseInWorld.pose.position.x)
        posMsg.data.append(poseInWorld.pose.position.y)
        posMsg.data.append(poseInWorld.pose.position.z)
        posMsg.data.append(rpy[0])
        posMsg.data.append(rpy[1])
        posMsg.data.append(rpy[2])
        self.velocity_goal_pub.publish(posMsg)

    def update_pos_control(self, x, y, z, roll, pitch, yaw, speed):
        targetPose = PoseStamped()
        targetPose.pose.position.x = x
        targetPose.pose.position.y = y
        targetPose.pose.position.z = z

        q = tf_transformations.quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))

        targetPose.pose.orientation.x = q[0]
        targetPose.pose.orientation.y = q[1]
        targetPose.pose.orientation.z = q[2]
        targetPose.pose.orientation.w = q[3]

        targetPose.header.frame_id = 'base_D_link'
        targetPose.header.stamp = rclpy.clock.Clock().now().to_msg()
        targetPoseInWorld = self.tf_buffer.transform(
            targetPose, 'world', timeout=rclpy.duration.Duration(seconds=1.0))

        target_rpy = tf_transformations.euler_from_quaternion([
            targetPoseInWorld.pose.orientation.x,
            targetPoseInWorld.pose.orientation.y,
            targetPoseInWorld.pose.orientation.z,
            targetPoseInWorld.pose.orientation.w,
        ])

        robotPose = PoseStamped()
        robotPose.header.frame_id = 'base_link'
        robotPose.header.stamp.sec = 0
        robotPose.header.stamp.nanosec = 0
        robotPoseInWorld = self.tf_buffer.transform(
            robotPose, 'world', timeout=rclpy.duration.Duration(seconds=1.0))

        current_rpy = tf_transformations.euler_from_quaternion([
            robotPoseInWorld.pose.orientation.x,
            robotPoseInWorld.pose.orientation.y,
            robotPoseInWorld.pose.orientation.z,
            robotPoseInWorld.pose.orientation.w,
        ])

        posVec = np.array([targetPoseInWorld.pose.position.x-robotPoseInWorld.pose.position.x,
                           targetPoseInWorld.pose.position.y-robotPoseInWorld.pose.position.y,
                           targetPoseInWorld.pose.position.z-robotPoseInWorld.pose.position.z])

        dist = np.linalg.norm(posVec)
        dirVec = posVec/dist

        if dist < 0.025:
            targetSpeed = speed*dist
        else:
            targetSpeed = speed

        moveVec = dirVec*targetSpeed

        self.send_vel_cmd(moveVec[0],
                          moveVec[1],
                          moveVec[2],
                          0,
                          0,
                          0)  # TODO add angular velocity support

    def send_vel_cmd(self, x, y, z, rx, ry, rz):
        velMsg = Float64MultiArray()
        velMsg.data.append(x)
        velMsg.data.append(y)
        velMsg.data.append(z)
        velMsg.data.append(rx)
        velMsg.data.append(ry)
        velMsg.data.append(rz)

        self.velocity_goal_pub.publish(velMsg)

    def stop(self) -> bool:
        self.state = RocapControlState.IDLE
        self.set_target(self.pose.position.x, self.pose.position.y,
                        self.pose.position.z)  # TODO add angular velocity support
        return True

    def disengage_brake(self) -> bool:
        self.state = RocapControlState.IDLE
        return True

    def engage_brake(self):
        self.stop()
        self.state = RocapControlState.ON_BRAKES
        return True


class RocapAPI(API):

    def __init__(self, base_url, permissions_file_path="permissions/signed_permissions.json"):
        self.base_url = base_url
        permissions_file = open(permissions_file_path)
        permissions = json.load(permissions_file)
        print("loaded permission file at :", permissions_file_path)

        with Client(self.base_url) as client:
            Accesskey = ApiV0AccessKey(
                signature=permissions["signature"], key=permissions["key"])
            self.token = request.sync(client=client, body=Accesskey).token

    def get_client(self):
        return Client(self.base_url).with_headers({"x-rocap-token": self.token})

    def set_position_goal(self, x, y, z, roll, pitch, yaw, speed) -> bool:

        request = LinearMoveRequest(position=Vec3(x, y, z),
                                    clamp=False,
                                    orientation=Vec3(roll, pitch, yaw),
                                    velocity=speed)

        with self.get_client() as client:
            response: Response = linear_move.sync(client=client, body=request)

            if (type(response) == LinearMoveResponse):
                print("The movement request is accepted.")
                return True
            elif type(response) == GenericError:
                print(response.cause)
                return False
            else:
                print("unexpected erreur append")
                return False

    def stop(self) -> bool:

        with self.get_client() as client:
            response: Response = stopAPI.sync(client=client)

            if (type(response) == CommandAck):
                print("The Stop request is accepted")
                return True
            elif type(response) == GenericError:
                print(response.cause)
                return False
            else:
                print("unexpected erreur append")
                return False

    def disengage_brake(self) -> bool:

        with self.get_client() as client:
            response = disengage_robot_brakes.sync_detailed(client=client)

            if response.status_code == HTTPStatus.ACCEPTED:
                print(
                    "brake disengage request accepted. The brakes will engage shortly.")
                return True
            elif response.status_code == HTTPStatus.FORBIDDEN:
                print("The Rocap cannot currently disengage its brakes.")
                return True
            else:
                print("The Rocap failed to process the disengage brake request.")
                return False

    def engage_brake(self) -> bool:

        with self.get_client() as client:
            response = engage_the_robot_brakes.sync(client=client)

            if (type(response) == CommandAck):
                print("Engage brakes request accepted. The brakes will engage shortly.")
                return True
            elif type(response) == GenericError:
                print(response.cause)
                return False
            else:
                print("The Rocap failed to process the engage brake request")
                return False

    def get_state(self) -> StateResponse | None:
        request = StateQuery(faults=True,
                             orientation=True,
                             state=True,
                             position=True)

        with self.get_client() as client:
            response: Response = state_query.sync(client=client, state=request)

            if (type(response) == StateResponse):

                if response.orientation == UNSET:
                    print("error orientation unset setting to default")
                    response.orientation = Vec3(0, 0, 45)

                if response.position == UNSET:
                    print("error position unset setting to default")
                    response.position = Vec3(0, 0, 0)

                return response

            elif type(response) == GenericError:
                print(response.cause)
                return None
            else:
                print("unexpected erreur append")
                return None


class API_bridge(Node):

    def __init__(self):

        super().__init__('MovingBaseController')

        self.declare_all_parameters()
        self.get_all_parameters()
        if self.useMock:
            self.robot = RocapAPIMock(self)
        else:
            self.robot = RocapAPI(
                base_url=self.base_url, permissions_file_path=self.permissions_file_path)

        self.tfBroadcaster = self.create_publisher(TFMessage, "/tf", 100)

        self.odomPublisher = self.create_publisher(Odometry, '/odom', 10)
        self.odomInfoPublisher = self.create_publisher(
            OdomInfo, '/odom_info', 10)
        self.pathPublisher = self.create_publisher(Path, "/path_filter", 10)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_tf)

        self.state = RocapControlState.IDLE
        self.stateMutex = threading.Condition(threading.RLock())

        # Move Server Variable Declaration
        self.moveMutex = threading.RLock()
        self.moveGoalHandle = None

        self.moveServer = ActionServer(self,
                                       Move,
                                       '/move_rocap',
                                       goal_callback=self.accept_move_goal,
                                       handle_accepted_callback=self.handle_accepted_move,
                                       execute_callback=self.execute_move,
                                       cancel_callback=self.cancel_move,
                                       callback_group=ReentrantCallbackGroup())

        # followPath Server Variable Declaration
        self.followPathMutex = threading.RLock()
        self.followPathGoalHandle = None

        self.followPathServer = ActionServer(self,
                                             FollowPath,
                                             '/follow_path',
                                             goal_callback=self.accept_follow_path_goal_action,
                                             handle_accepted_callback=self.handle_accepted_follow_path_action,
                                             execute_callback=self.execute_follow_path_action,
                                             cancel_callback=self.cancel_follow_path_action,
                                             callback_group=ReentrantCallbackGroup())

        self.moveClient = ActionClient(self,
                                       Move,
                                       '/move_rocap',
                                       callback_group=MutuallyExclusiveCallbackGroup())

        self.odomPose = None

        self.robot.disengage_brake()

    def declare_all_parameters(self):
        self.declare_parameter('base_url', 'http://192.168.100.201:5577')
        self.declare_parameter('move_timeout', 120.0)
        self.declare_parameter('permissions_file_path',
                               'permissions/signed_permissions.json')
        self.declare_parameter('use_mock', False)

    def get_all_parameters(self):
        self.base_url = self.get_parameter(
            'base_url').get_parameter_value().string_value
        self.moveTimeOut = self.get_parameter(
            'move_timeout').get_parameter_value().double_value
        self.permissions_file_path = self.get_parameter(
            'permissions_file_path').get_parameter_value().string_value
        self.useMock = self.get_parameter(
            'use_mock').get_parameter_value().bool_value

    def publish_tf(self):
        state: StateResponse = self.robot.get_state()
        if state != None:

            self.stateMutex.acquire()
            self.state: RocapControlState = state.state
            self.stateMutex.notify_all()
            self.stateMutex.release()
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = state.position.x
            t.transform.translation.y = state.position.y
            t.transform.translation.z = state.position.z

            q = tf_transformations.quaternion_from_euler(
                state.orientation.x, state.orientation.y, state.orientation.z)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Send the transformation
            self.tfBroadcaster.publish(TFMessage(transforms=[t]))
            self.publish_odometry(t)

    def publish_odometry(self, transform: TransformStamped):
        msg = Odometry()
        msg.header = transform.header
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = transform.transform.translation.x
        msg.pose.pose.position.y = transform.transform.translation.y
        msg.pose.pose.position.z = transform.transform.translation.z
        msg.pose.pose.orientation.x = transform.transform.rotation.x
        msg.pose.pose.orientation.y = transform.transform.rotation.y
        msg.pose.pose.orientation.z = transform.transform.rotation.z
        msg.pose.pose.orientation.w = transform.transform.rotation.w
        self.odomPose = msg.pose.pose
        self.odomPublisher.publish(msg)

        self.odomInfoPublisher.publish(OdomInfo())

    def send_position_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        roll = 0
        pitch = 0
        yaw = 45

        if self.robot.set_position_goal(x, y, z, roll, pitch, yaw, 0.2):
            print("goal set to x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f" %
                  (x, y, z, roll, pitch, yaw))
            return True
        else:
            return False

    def accept_move_goal(self, goal_request: Move.Goal) -> GoalResponse:
        """Accept or reject a client request to begin a Move action."""
        with self.moveMutex:
            if self.moveGoalHandle is not None and self.moveGoalHandle.is_active:
                self.get_logger().info(
                    'Received moving request while already moving stopping for security ...')
                self.moveGoalHandle.abort()
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_move(self, goal_handle: ServerGoalHandle) -> None:
        """Start execution of an already accepted goal.
           If an other goal is being executed cancel the to goal for security
        """
        with self.moveMutex:
            if self.moveGoalHandle is not None and self.moveGoalHandle.is_active:
                self.get_logger().info(
                    'Received moving request while already moving stopping for security ...')
                self.moveGoalHandle.abort()
            else:
                self.moveGoalHandle = goal_handle
                self.moveGoalHandle.execute()

    def cancel_move(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept or reject a client request to cancel a Move action."""

        self.get_logger().info('Reseved a cancelation request for current movement')
        return CancelResponse.ACCEPT

    def execute_move(self, goal_handle: ServerGoalHandle) -> Move.Result:
        """Execute a client request of the action move"""

        goal: PoseStamped = goal_handle.request.target

        if self.send_position_goal(goal):
            self.get_logger().info(
                'Moving to Target point {0}...'.format(goal.pose.position))
            startTime = time.time()

            time.sleep(0.25)  # TODO change the wating time base on test
            while time.time() - startTime < self.moveTimeOut:  # TODO Optimaze State Handling

                if not goal_handle.is_active:
                    self.get_logger().info('Aborting moving goal ...')
                    self.robot.stop()
                    return Move.Result()

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Cancelling moving goal ...')
                    self.robot.stop()
                    goal_handle.canceled()
                    return Move.Result()

                with self.stateMutex:
                    self.stateMutex.wait()
                    if self.state == RocapControlState.IDLE:
                        goal_handle.succeed()
                        return Move.Result()
            self.get_logger().info('Timeout reach abort mouvement...')

            # timeout reach abort
            self.robot.stop()
            goal_handle.abort()
            return Move.Result()

        else:
            self.get_logger().info('Move request failed...')
            goal_handle.abort()
            return Move.Result()

    def accept_follow_path_goal_action(self, goal_request: FollowPath.Goal) -> GoalResponse:
        """Accept or reject a client request to begin a follow_path action."""
        with self.followPathMutex:
            if self.followPathGoalHandle is not None and self.followPathGoalHandle.is_active:
                self.get_logger().info(
                    'Received follow path request while already moving stopping for security ...')
                self.followPathGoalHandle.abort()
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_follow_path_action(self, goal_handle: ServerGoalHandle) -> None:
        """Start execution of an already accepted goal.
           If an other goal is being executed cancel all goal for security
        """
        with self.followPathMutex:
            if self.followPathGoalHandle is not None and self.followPathGoalHandle.is_active:
                self.get_logger().info(
                    'Received followPath request while already moving stopping for security ...')
                self.followPathGoalHandle.abort()
            else:
                self.followPathGoalHandle = goal_handle
                self.followPathGoalHandle.execute()

    def cancel_follow_path_action(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept or reject a client request to cancel a FollowPath action."""

        self.get_logger().info('Received a cancelation request for current FollowPath action')
        return CancelResponse.ACCEPT

    def execute_follow_path_action(self, goal_handle: ServerGoalHandle) -> Move.Result:
        """Execute a client request of the FollowPath action"""

        def follow_path_stop_condition(
        ): return not goal_handle.is_active or goal_handle.is_cancel_requested
        path: Path = goal_handle.request.target
        pathToPublish = copy.deepcopy(path)
        for ctr in range(0, len(path.poses)):
            targetPose: Pose = path.poses[ctr]
            pathToPublish.poses = copy.deepcopy(
                path.poses[max(0, ctr-1):])  # to verify

            if self.odomPose != None:
                pathToPublish.poses[0].pose = self.odomPose

            self.get_logger().info(
                'Going to new point({0})'.format(len(pathToPublish.poses)))
            self.pathPublisher.publish(pathToPublish)
            self.get_logger().info(
                'Going to new point({0}/{1})'.format(ctr, len(path.poses)))

            if follow_path_stop_condition():
                return self.terminate_execute_follow_path_action(goal_handle)

            if not self.callMove(targetPose, follow_path_stop_condition):
                return self.terminate_execute_follow_path_action(goal_handle)

        self.get_logger().info("Follow path succed")
        goal_handle.succeed()
        return FollowPath.Result()

    def terminate_execute_follow_path_action(self, goal_handle: ServerGoalHandle) -> FollowPath.Result:

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info("Canceling FollowPath...")

        else:
            if goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().info("Aborting FollowPath...")

        return FollowPath.Result()

    def callMove(self, targetPose: Pose, stopCondition) -> bool:

        if not self.moveClient.wait_for_server(20):
            self.get_logger().info("No move action server available aborting...")
            return False

        goal_msg = Move.Goal()
        goal_msg.target = targetPose
        goal_msg.target.header.stamp = self.get_clock().now().to_msg()

        moveActionFuture: Future = self.moveClient.send_goal_async(goal_msg)

        if not self.waitForFutureAsync(moveActionFuture, failCondition=stopCondition):
            moveActionFuture.cancel()
            return False

        clientGoalHandle: ClientGoalHandle = moveActionFuture.result()
        if not clientGoalHandle.accepted:
            self.get_logger().info('moving request to point {0} refuse aborting...'.format(
                targetPose.pose.position))
            return False

        moveResutFuture: Future = clientGoalHandle.get_result_async()
        if not self.waitForFutureAsync(moveResutFuture, stopCondition):
            clientGoalHandle.cancel_goal()
            return False

        moveResult: Move.Result = moveResutFuture.result()
        if moveResult.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'point {0} not reached aborting...'.format(targetPose.pose.position))
            return False

        return True

    @staticmethod
    def waitForFutureAsync(future: Future, failCondition=lambda: False, waitTime=0.25, timeout=120) -> bool:
        startTime = time.time()
        while time.time() - startTime < timeout:  # TODO Optimaze State Handling
            time.sleep(waitTime)
            if future.done():
                return True

            if failCondition():
                return False

        return False

    # Deleting (Calling destructor)

    def __del__(self):
        self.robot.engage_brake()


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    minimal_subscriber = API_bridge()
    executor.add_node(minimal_subscriber)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
