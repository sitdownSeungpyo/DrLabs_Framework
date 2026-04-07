// Package bridge provides a gRPC bridge between ROS 2 and the motor SDK.
// This enables ROS 2 nodes to control motors without CGo dependencies.
//
// Architecture:
//
//	ROS 2 topics                  dxl_go process
//	/joint_commands  ──gRPC──>   gRPC Server (Go)
//	/joint_states    <─gRPC──    motor.Controller
//	  ros2_bridge_node
//	  (Python, ~100 lines)
//
// The gRPC service definition will be in proto/motor.proto:
//
//	service MotorService {
//	    rpc SetGoals(GoalsRequest) returns (GoalsResponse);
//	    rpc StreamFeedback(FeedbackRequest) returns (stream FeedbackResponse);
//	    rpc SetOperatingMode(ModeRequest) returns (ModeResponse);
//	    rpc ExecuteTrajectory(TrajectoryRequest) returns (stream Progress);
//	}
package bridge
