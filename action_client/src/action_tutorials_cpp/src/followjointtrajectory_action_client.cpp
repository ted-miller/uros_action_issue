// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "rcl/rcl.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.h" //C version

using FJT = control_msgs::action::FollowJointTrajectory;
using FJT_Result = control_msgs::action::FollowJointTrajectory_Result;
using FJT_Goal = control_msgs::action::FollowJointTrajectory_Goal;

using namespace std;

shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = FJT_Result::SUCCESSFUL;

void common_goal_response(
    shared_future<rclcpp_action::ClientGoalHandle<FJT>::SharedPtr> future)
{
    RCLCPP_DEBUG(
        node->get_logger(), "common_goal_response time: %f",
        rclcpp::Clock().now().seconds());
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        common_goal_accepted = false;
        printf("Goal rejected\n");
    }
    else
    {
        common_goal_accepted = true;
        printf("Goal accepted\n");
    }
}

void common_result_response(
    const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult &result)
{
    printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
    common_resultcode = result.code;
    common_action_result_code = result.result->error_code;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        printf("SUCCEEDED result code\n");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        printf("Goal was aborted\n");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        printf("Goal was canceled\n");
        return;
    default:
        printf("Unknown result code\n");
        return;
    }
}

void common_feedback(
    rclcpp_action::ClientGoalHandle<FJT>::SharedPtr,
    const shared_ptr<const FJT::Feedback> feedback)
{
    cout << "feedback->desired.positions :";
    for (auto &x : feedback->desired.positions)
    {
        cout << x << "\t";
    }
    cout << endl;
    cout << "feedback->desired.velocities :";
    for (auto &x : feedback->desired.velocities)
    {
        cout << x << "\t";
    }
    cout << endl;
}

void get_current_position(trajectory_msgs::msg::JointTrajectoryPoint* point)
{    
    rcl_init_options_t initOptions = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&initOptions, rcutils_get_default_allocator());
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_init(0, NULL, &initOptions, &context);
    rcl_init_options_fini(&initOptions);

    cout << "rcl init" << endl;

    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t nodeOptions = rcl_node_get_default_options();
    rcl_node_init(&node, "get_position", "", &context, &nodeOptions);
    cout << "rcl_node_init" << endl;

    const rosidl_message_type_support_t *typeSupport = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscriptionOptions = rcl_subscription_get_default_options();
    rcl_subscription_init(&subscription, &node, typeSupport, "/grp0/joint_states", &subscriptionOptions);
    cout << "rcl_subscription_init" << endl;

    sensor_msgs__msg__JointState* msg;
    rmw_message_info_t messageInfo;

    msg = sensor_msgs__msg__JointState__create();
    rosidl_runtime_c__String__Sequence__init(&msg->name, point->positions.size());
    rosidl_runtime_c__double__Sequence__init(&msg->position, point->positions.size());
    rosidl_runtime_c__double__Sequence__init(&msg->effort, point->positions.size());
    rosidl_runtime_c__double__Sequence__init(&msg->velocity, point->positions.size());

    rcl_ret_t ret = rcl_take(&subscription, msg, &messageInfo, NULL);
    cout << "rcl_take: " << ret << endl;

    cout << " msg size = " << msg->position.size << endl;
    point->positions[0] = msg->position.data[0];
    point->positions[1] = msg->position.data[1];
    point->positions[2] = msg->position.data[2];
    point->positions[3] = msg->position.data[3];
    point->positions[4] = msg->position.data[4];
    point->positions[5] = msg->position.data[5];

    cout << "before rcl_subscription_fini" << endl;
    rcl_subscription_fini(&subscription, &node);
    cout << "rcl_subscription_fini" << endl;
    rcl_node_fini(&node);
    cout << "rcl_node_fini" << endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    node = make_shared<rclcpp::Node>("trajectory_test_node");

    cout << "node created" << endl;

    rclcpp_action::Client<FJT>::SharedPtr action_client;
    action_client = rclcpp_action::create_client<FJT>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_logging_interface(),
        node->get_node_waitables_interface(),
        "/follow_joint_trajectory");

    bool response =
        action_client->wait_for_action_server(chrono::seconds(1));
    if (!response)
    {
        throw runtime_error("could not get action server");
    }
    cout << "Created action server" << endl;

    vector<string> joint_names = {
        "joint_0",
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
    };

    vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    trajectory_msgs::msg::JointTrajectoryPoint startPoint;
    startPoint.time_from_start = rclcpp::Duration::from_seconds(0.0); // start asap
    startPoint.positions.resize(joint_names.size());

    cout << "Getting current position" << endl;
    get_current_position(&startPoint);
    cout << "Got current position" << endl;

    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(1.0);
    point2.positions = startPoint.positions;
    point2.positions[3] += 0.349066;

    trajectory_msgs::msg::JointTrajectoryPoint point3;
    point3.time_from_start = rclcpp::Duration::from_seconds(2.0);
    point3.positions = startPoint.positions;
    point3.positions[3] -= 0.349066;

    trajectory_msgs::msg::JointTrajectoryPoint point4;
    point4.time_from_start = rclcpp::Duration::from_seconds(3.0);
    point4.positions.resize(joint_names.size());
    point4.positions = startPoint.positions;

    cout << "pushing points" << endl;
    points.push_back(startPoint);
    points.push_back(point2);
    points.push_back(point3);
    points.push_back(point4);

    rclcpp_action::Client<FJT>::SendGoalOptions opt;
    opt.goal_response_callback = bind(common_goal_response, placeholders::_1);
    opt.result_callback = bind(common_result_response, placeholders::_1);
    opt.feedback_callback = bind(common_feedback, placeholders::_1, placeholders::_2);

    FJT_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    goal_msg.trajectory.joint_names = joint_names;
    goal_msg.trajectory.points = points;

    cout << "sending goal with " << goal_msg.trajectory.points.size() << " points" << endl;

        auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

    if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "send goal call ok :)");

    rclcpp_action::ClientGoalHandle<FJT>::SharedPtr
        goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Goal was accepted by server");

    // Wait for the server to be done with the goal
    auto result_future = action_client->async_get_result(goal_handle);
    RCLCPP_INFO(node->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
        return 1;
    }

    cout << "async_send_goal" << endl;
    rclcpp::shutdown();

    return 0;
}