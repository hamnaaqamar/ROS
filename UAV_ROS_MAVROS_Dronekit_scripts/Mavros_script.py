#!/usr/bin/env python3
"""
ROS1/MAVROS script for mission 3
"""
import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import socket
import json
import math

class CVPositioningNode:
    def __init__(self):
        rospy.init_node('cv_positioning_node')
        
        self.current_state = State()
        self.normalized_coords = None
        self.active = False
        self.centered = False
        self.center_threshold = 0.05  
        self.centered_count = 0  
        self.centered_threshold = 10  
        
        self.socket_port = 5000
        self.setup_socket()
        
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.vision_sub = rospy.Subscriber('/vision/normalized', PoseStamped, self.vision_cb)
        
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        
        rospy.loginfo("MAVROS connected")
    
    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('localhost', self.socket_port))
        rospy.loginfo("Connected to DroneKit script")
    
    def state_cb(self, msg):
        self.current_state = msg
    
    def vision_cb(self, msg):
        t = rospy.Time.now().to_sec()
        radius = min(0.4, t/10)  
        x = 0.5 + radius * math.sin(t)
        y = 0.5 + radius * math.cos(t)
        
        self.normalized_coords = (x, y)
        if self.active:
            self.adjust_position()

    
    def adjust_position(self):
        if not self.normalized_coords:
            return
        
        x, y = self.normalized_coords
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time.now()
    
        x_error = x - 0.5
        y_error = y - 0.5
        
        if abs(x_error) < self.center_threshold and abs(y_error) < self.center_threshold:
            self.centered_count += 1
            if self.centered_count >= self.centered_threshold and not self.centered:
                self.centered = True
                self.send_centered_signal()
            return
        else:
            self.centered_count = 0  
            
        # Proportional control for velocity commands
        kp = 0.5  # Proportional gain
        max_speed = 0.5  
        
        vel_x = -kp * y_error 
        vel_y = -kp * x_error  
        
        vel_x = max(-max_speed, min(max_speed, vel_x))
        vel_y = max(-max_speed, min(max_speed, vel_y))
        
        vel_cmd.twist.linear.x = vel_x
        vel_cmd.twist.linear.y = vel_y
        self.vel_pub.publish(vel_cmd)
    
    def send_centered_signal(self):
        response = {
            "command": "lower_altitude",
            "value": 1.0  
        }
        
        try:
            self.sock.send(json.dumps(response).encode())
            rospy.loginfo("Sent lower altitude command")
            self.active = False  
        except Exception as e:
            rospy.logerr(f"Socket error: {e}")
    
    def run(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            try:
                data = self.sock.recv(1024, socket.MSG_DONTWAIT)
                if data:
                    command = json.loads(data.decode())
                    if command.get("command") == "start_cv":
                        self.active = True
                        self.centered = False
                        self.centered_count = 0
                        rospy.loginfo("Starting CV positioning")
            except BlockingIOError:
                pass  
            except Exception as e:
                rospy.logerr(f"Error processing command: {e}")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        node = CVPositioningNode()
        node.run()
    except rospy.ROSInterruptException:
        pass