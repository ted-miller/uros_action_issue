
#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rmw/rmw.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/init_options.h>

#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <control_msgs/action/follow_joint_trajectory.h>

#include <stdio.h>
#include <unistd.h>

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define AGENT_IP "127.0.0.1"
#define AGENT_PORT "2018"

#define BOOL int
#define TRUE 1
#define FALSE 0

#define MAX_DEBUG_MESSAGE_SIZE 1024

rcl_init_options_t rclInitOptions;
rclc_support_t rclSupport;
rcl_node_t rclNode;

rcl_action_server_t actionServerFollowJointTrajectory;
rcl_wait_set_t waitSetMotionServer;

BOOL bMotionServer_FollowJointTrajectory_Active;
control_msgs__action__FollowJointTrajectory_FeedbackMessage* feedback_FollowJointTrajectory;
control_msgs__action__FollowJointTrajectory_SendGoal_Request* Ros_MotionControl_FollowJointTrajectory_Request;

BOOL Ros_Controller_IsInMotion()
{
	return false;
}

void Ros_Debug_BroadcastData(char *fmt, ...)
{
	char str[MAX_DEBUG_MESSAGE_SIZE];
	va_list va;

#warning add a timestamp to this message;

	memset(str, 0x00, MAX_DEBUG_MESSAGE_SIZE);

	va_start(va, fmt);
	vsnprintf(str, MAX_DEBUG_MESSAGE_SIZE, fmt, va);
	va_end(va);

	puts(str);
}

void Ros_MotionServer_CheckForActivity_FollowJointTrajectory(rcl_wait_set_t *wait_set)
{
	static rcl_action_goal_handle_t* goal_handle;
	static rcl_action_goal_info_t goal_info;
	rcl_ret_t ret;

	bool is_goal_request_ready = false;
	bool is_cancel_request_ready = false;
	bool is_result_request_ready = false;
	bool is_goal_expired = false;

	//==================================
	rcl_action_server_wait_set_get_entities_ready(wait_set, &actionServerFollowJointTrajectory,
													&is_goal_request_ready, &is_cancel_request_ready, &is_result_request_ready, &is_goal_expired);

	//==================================
	//INCOMING REQUEST
	if (is_goal_request_ready)
	{
		Ros_Debug_BroadcastData("FollowJointTrajectory - Goal request received");

		rmw_request_id_t request_header;
		control_msgs__action__FollowJointTrajectory_SendGoal_Request* pending_ros_goal_request;

		pending_ros_goal_request = control_msgs__action__FollowJointTrajectory_SendGoal_Request__create();

		RCSOFTCHECK(rcl_action_take_goal_request(&actionServerFollowJointTrajectory, &request_header, pending_ros_goal_request))

		control_msgs__action__FollowJointTrajectory_SendGoal_Response* ros_goal_response;
		ros_goal_response = control_msgs__action__FollowJointTrajectory_SendGoal_Response__create();

		//ros_goal_response->accepted = ((Ros_MotionControl_FollowJointTrajectory_Request == NULL) && Ros_Controller_IsMotionReady() && Ros_MotionControl_InitTrajPointFull(pending_ros_goal_request));
		ros_goal_response->accepted = (pending_ros_goal_request != NULL && pending_ros_goal_request->goal.trajectory.points.size >= 1);
		if (pending_ros_goal_request)
			Ros_Debug_BroadcastData("pending_ros_goal_request->goal.trajectory.points.size = %d", pending_ros_goal_request->goal.trajectory.points.size);
		else
			Ros_Debug_BroadcastData("pending_ros_goal_request");

		rcl_action_send_goal_response(&actionServerFollowJointTrajectory, &request_header, ros_goal_response);
		Ros_Debug_BroadcastData("FollowJointTrajectory - Goal response");

		if (ros_goal_response->accepted) 
		{
			Ros_Debug_BroadcastData("FollowJointTrajectory - Goal request accepted");

			// ---- Accept goal
			goal_info = rcl_action_get_zero_initialized_goal_info();
			goal_info.goal_id = pending_ros_goal_request->goal_id;
			//goal_info.stamp = 
			goal_handle = rcl_action_accept_new_goal(&actionServerFollowJointTrajectory, &goal_info);

			// ---- Update state
			rcl_action_update_goal_state(goal_handle, GOAL_EVENT_EXECUTE);

			// ---- Publish statuses
			rcl_action_goal_status_array_t c_status_array = rcl_action_get_zero_initialized_goal_status_array();
			rcl_action_get_goal_status_array(&actionServerFollowJointTrajectory, &c_status_array);
			rcl_action_publish_status(&actionServerFollowJointTrajectory, &c_status_array.msg);

			// ---- Pass off the data to MotionControl
			Ros_MotionControl_FollowJointTrajectory_Request = pending_ros_goal_request; //will be destroyed in MotionControl
			bMotionServer_FollowJointTrajectory_Active = TRUE;

			// ---- Build feedback message
			feedback_FollowJointTrajectory = control_msgs__action__FollowJointTrajectory_FeedbackMessage__create();
			feedback_FollowJointTrajectory->goal_id = goal_info.goal_id;

			//copy string sequence from the request message
			int numberOfJointNames = pending_ros_goal_request->goal.trajectory.joint_names.size;
			rosidl_runtime_c__String__Sequence__init(&feedback_FollowJointTrajectory->feedback.joint_names, numberOfJointNames);
			for (int i = 0; i < numberOfJointNames; i += 1)
			{
				rosidl_runtime_c__String__assign(&feedback_FollowJointTrajectory->feedback.joint_names.data[i], pending_ros_goal_request->goal.trajectory.joint_names.data[i].data);
			}
		}
		else
		{
			Ros_Debug_BroadcastData("FollowJointTrajectory - Goal request rejected");

		}
		//cleanup memory
		control_msgs__action__FollowJointTrajectory_SendGoal_Request__destroy(pending_ros_goal_request);
		control_msgs__action__FollowJointTrajectory_SendGoal_Response__destroy(ros_goal_response);
	}
	
	//==================================
	//PERFORMING ACTION
	if (bMotionServer_FollowJointTrajectory_Active)
	{
		// ---- Publish feedback
		#warning: populate feedback_FollowJointTrajectory;
		rcl_action_publish_feedback(&actionServerFollowJointTrajectory, feedback_FollowJointTrajectory);
	}

	//==================================
	//NOTIFY ACTION COMPLETE
	if (bMotionServer_FollowJointTrajectory_Active && (Ros_MotionControl_FollowJointTrajectory_Request == NULL) && !Ros_Controller_IsInMotion())
	{
		// ---- Sending result ready
		Ros_Debug_BroadcastData("FollowJointTrajectory - Notify completion");

		#warning i need to check the tolerances and whatnot to determine success;
		rcl_action_update_goal_state(goal_handle, GOAL_EVENT_SUCCEED);

		rcl_action_notify_goal_done(&actionServerFollowJointTrajectory);

		bMotionServer_FollowJointTrajectory_Active = FALSE;
		control_msgs__action__FollowJointTrajectory_FeedbackMessage__destroy(feedback_FollowJointTrajectory);
		feedback_FollowJointTrajectory = NULL;
	}
	
	//==================================
	//SEND FINAL RESULTS
	if (is_result_request_ready && !bMotionServer_FollowJointTrajectory_Active)
	{
		Ros_Debug_BroadcastData("FollowJointTrajectory - Send result");

		control_msgs__action__FollowJointTrajectory_GetResult_Request ros_result_request;
		rmw_request_id_t request_header;
		rcl_action_take_result_request(&actionServerFollowJointTrajectory, &request_header, &ros_result_request);

		control_msgs__action__FollowJointTrajectory_GetResult_Response ros_result_response;
		control_msgs__action__FollowJointTrajectory_Result result;

		#warning set this to the actual value, based on success;
		result.error_code = control_msgs__action__FollowJointTrajectory_Result__SUCCESSFUL;
		rosidl_runtime_c__String__assign(&result.error_string, "All good, dawg!");

		ros_result_response.status = action_msgs__msg__GoalStatus__STATUS_SUCCEEDED;
		ros_result_response.result = result;
		
		rcl_action_send_result_response(&actionServerFollowJointTrajectory, &request_header, &ros_result_response);
	}

	//==================================
	//ABORT MOTION (canceled)
	if (is_cancel_request_ready)
	{
		#warning do something here;

		bMotionServer_FollowJointTrajectory_Active = FALSE;
		control_msgs__action__FollowJointTrajectory_FeedbackMessage__destroy(feedback_FollowJointTrajectory);
		feedback_FollowJointTrajectory = NULL;
	}

	//==================================
	//ABORT MOTION (expired)
	if (is_goal_expired)
	{
		#warning do something here;

		bMotionServer_FollowJointTrajectory_Active = FALSE;
		control_msgs__action__FollowJointTrajectory_FeedbackMessage__destroy(feedback_FollowJointTrajectory);
		feedback_FollowJointTrajectory = NULL;
	}
}

void Ros_MotionServer_StartMotionServices()
{
	int groupNo;
	int connectionIndex;

	//-------------------------------------------
	while (TRUE) //will be terminated in mpMain when agent disconnects
	{
		usleep(20000);

		//check for activity on the server
		rcl_wait_set_clear(&waitSetMotionServer);

		size_t index;
		rcl_action_wait_set_add_action_server(&waitSetMotionServer, &actionServerFollowJointTrajectory, &index);

		//Don't use a timeout of zero. It doesn't relinquish control of the CPU and sleep.
		rcl_wait(&waitSetMotionServer, /*0*/ RCL_MS_TO_NS(1));

		//--------------------------------
		Ros_MotionServer_CheckForActivity_FollowJointTrajectory(&waitSetMotionServer);
	}
}

int main()
{
	rcl_ret_t ret;
	rcl_allocator_t allocator;

	allocator = rcl_get_default_allocator();

	rclInitOptions = rcl_get_zero_initialized_init_options();
	RCSOFTCHECK(rcl_init_options_init(&rclInitOptions, allocator))
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&rclInitOptions);
	
	ret = RCL_RET_ERROR;
	do
	{
		usleep(1000000);

		rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options);

		ret = rmw_uros_ping_agent_options(1000, 1, rmw_options);

	} while (ret != RCL_RET_OK);
	
	//==================================
	//create node
	RCSOFTCHECK(rclc_support_init_with_options(&rclSupport, 0, NULL, &rclInitOptions, &allocator))
	RCSOFTCHECK(rclc_node_init_default(&rclNode, "action_server", "", &rclSupport))

	//==================================
	//initialize action server - FollowJointTrajectory
	const rosidl_action_type_support_t* action_type_support = ROSIDL_GET_ACTION_TYPE_SUPPORT(control_msgs, FollowJointTrajectory);
	actionServerFollowJointTrajectory = rcl_action_get_zero_initialized_server();
	rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

	RCSOFTCHECK(rcl_action_server_init(&actionServerFollowJointTrajectory, &rclNode, &rclSupport.clock, action_type_support, "follow_joint_trajectory", &action_server_ops))

	Ros_MotionControl_FollowJointTrajectory_Request = NULL;
	bMotionServer_FollowJointTrajectory_Active = FALSE;
	feedback_FollowJointTrajectory = NULL;

	size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;
	ret = rcl_action_server_wait_set_get_num_entities(&actionServerFollowJointTrajectory,
		&num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services);

	waitSetMotionServer = rcl_get_zero_initialized_wait_set();
	
	RCSOFTCHECK(rcl_wait_set_init(&waitSetMotionServer, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &rclSupport.context, allocator))

	Ros_MotionServer_StartMotionServices();
}
