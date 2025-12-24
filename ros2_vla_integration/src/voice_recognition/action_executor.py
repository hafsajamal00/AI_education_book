"""
ROS 2 Action Client for VLA System
Implements communication with ROS 2 for robot action execution
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import asyncio
from typing import Dict, Any, List


class VLAActionExecutor(Node):
    """
    Node that executes action sequences received from the VLA API
    """
    def __init__(self):
        super().__init__('vla_action_executor')
        
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for manipulation commands (if available)
        self.manipulation_publisher = self.create_publisher(String, '/manipulation_commands', 10)
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, '/vla_status', 10)
        
        self.get_logger().info('VLA Action Executor initialized')

    def execute_action_sequence(self, action_sequence: Dict[str, Any], simulation_env_id: str):
        """
        Execute a sequence of actions in the simulation environment
        """
        self.get_logger().info(f"Executing action sequence {action_sequence['id']} in simulation {simulation_env_id}")
        
        # Update status
        status_msg = String()
        status_msg.data = f"Starting execution of sequence {action_sequence['id']}"
        self.status_publisher.publish(status_msg)
        
        try:
            for action in sorted(action_sequence.get("sequence", []), key=lambda x: x["order"]):
                self.get_logger().info(f"Executing action: {action['type']} with params: {action['parameters']}")
                
                # Execute the action based on its type
                success = self.execute_single_action(action)
                
                if not success:
                    self.get_logger().error(f"Failed to execute action: {action['type']}")
                    break
                
                # Small delay between actions
                time.sleep(0.5)
            
            # Update status when sequence is complete
            status_msg.data = f"Completed execution of sequence {action_sequence['id']}"
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error executing action sequence: {str(e)}")
            status_msg.data = f"Error in sequence {action_sequence['id']}: {str(e)}"
            self.status_publisher.publish(status_msg)

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action
        """
        action_type = action["type"]
        params = action["parameters"]
        
        if action_type == "move_forward":
            return self.move_forward(params)
        elif action_type == "move_backward":
            return self.move_backward(params)
        elif action_type == "turn_left":
            return self.turn_left(params)
        elif action_type == "turn_right":
            return self.turn_right(params)
        elif action_type == "pick_up":
            return self.pick_up_object(params)
        elif action_type == "place":
            return self.place_object(params)
        elif action_type == "unknown":
            self.get_logger().warn(f"Unknown command: {params.get('text', 'no text')}")
            return False
        else:
            self.get_logger().warn(f"Unsupported action type: {action_type}")
            return False

    def move_forward(self, params: Dict[str, Any]) -> bool:
        """
        Move the robot forward
        """
        try:
            msg = Twist()
            msg.linear.x = 0.5  # Forward speed
            msg.angular.z = 0.0  # No rotation
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info('Moving forward')
            
            # Stop after a short duration
            time.sleep(1.0)
            self.stop_robot()
            return True
        except Exception as e:
            self.get_logger().error(f"Error in move_forward: {str(e)}")
            return False

    def move_backward(self, params: Dict[str, Any]) -> bool:
        """
        Move the robot backward
        """
        try:
            msg = Twist()
            msg.linear.x = -0.5  # Backward speed
            msg.angular.z = 0.0  # No rotation
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info('Moving backward')
            
            # Stop after a short duration
            time.sleep(1.0)
            self.stop_robot()
            return True
        except Exception as e:
            self.get_logger().error(f"Error in move_backward: {str(e)}")
            return False

    def turn_left(self, params: Dict[str, Any]) -> bool:
        """
        Turn the robot left
        """
        try:
            msg = Twist()
            msg.linear.x = 0.0  # No forward movement
            msg.angular.z = 0.5  # Left turn speed
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info('Turning left')
            
            # Stop after a short duration
            time.sleep(0.8)  # Adjust for 90-degree turn
            self.stop_robot()
            return True
        except Exception as e:
            self.get_logger().error(f"Error in turn_left: {str(e)}")
            return False

    def turn_right(self, params: Dict[str, Any]) -> bool:
        """
        Turn the robot right
        """
        try:
            msg = Twist()
            msg.linear.x = 0.0  # No forward movement
            msg.angular.z = -0.5  # Right turn speed
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info('Turning right')
            
            # Stop after a short duration
            time.sleep(0.8)  # Adjust for 90-degree turn
            self.stop_robot()
            return True
        except Exception as e:
            self.get_logger().error(f"Error in turn_right: {str(e)}")
            return False

    def pick_up_object(self, params: Dict[str, Any]) -> bool:
        """
        Simulate picking up an object
        """
        try:
            object_name = params.get('object', 'unknown')
            self.get_logger().info(f'Attempting to pick up: {object_name}')
            
            # In a real implementation, this would call a manipulation action server
            # For simulation, we'll just publish a command
            cmd_msg = String()
            cmd_msg.data = f"pick_up:{object_name}"
            self.manipulation_publisher.publish(cmd_msg)
            
            time.sleep(1.0)  # Simulate time to pick up object
            return True
        except Exception as e:
            self.get_logger().error(f"Error in pick_up_object: {str(e)}")
            return False

    def place_object(self, params: Dict[str, Any]) -> bool:
        """
        Simulate placing an object
        """
        try:
            self.get_logger().info('Placing object')
            
            # In a real implementation, this would call a manipulation action server
            # For simulation, we'll just publish a command
            cmd_msg = String()
            cmd_msg.data = "place_object"
            self.manipulation_publisher.publish(cmd_msg)
            
            time.sleep(1.0)  # Simulate time to place object
            return True
        except Exception as e:
            self.get_logger().error(f"Error in place_object: {str(e)}")
            return False

    def stop_robot(self):
        """
        Stop the robot by publishing zero velocities
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)


def execute_action_sequence_from_api(action_sequence: Dict[str, Any], simulation_env_id: str):
    """
    Function to execute an action sequence received from the API
    This would be called from the background task in the API
    """
    rclpy.init()
    
    executor = VLAActionExecutor()
    
    # Execute the action sequence
    executor.execute_action_sequence(action_sequence, simulation_env_id)
    
    # Shutdown ROS
    executor.destroy_node()
    rclpy.shutdown()


# Example usage when running directly
if __name__ == '__main__':
    # Example action sequence
    example_sequence = {
        "id": "test-sequence-123",
        "sequence": [
            {
                "id": "action-1",
                "type": "move_forward",
                "parameters": {},
                "order": 0
            },
            {
                "id": "action-2", 
                "type": "turn_right",
                "parameters": {},
                "order": 1
            },
            {
                "id": "action-3",
                "type": "move_forward",
                "parameters": {},
                "order": 2
            }
        ],
        "status": "pending",
        "created_at": "2023-01-01T00:00:00Z"
    }
    
    execute_action_sequence_from_api(example_sequence, "gazebo_sim_1")