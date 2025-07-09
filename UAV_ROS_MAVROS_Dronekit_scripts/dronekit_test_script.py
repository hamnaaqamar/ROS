#!/usr/bin/env python
"""
DroneKit mission control script for UAV payload transport
"""
import dronekit
import time
import socket
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math


CONNECTION_STRING = 'tcp:127.0.0.1:5760'  # ADD VM IP ADDRESS
PICKUP_HEX_COORDS = (34.0670230,72.6428461)  
DROPOFF_TRI_COORDS = (34.0678940,72.6422238) 
CRUISE_ALTITUDE = 15  
DESCENT_ALTITUDE = 3  
SOCKET_PORT = 5000

def connect_vehicle():
    print("Connecting to vehicle...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    return vehicle

def arm_and_takeoff(vehicle, altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(altitude)
    
    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_location(vehicle, location, groundspeed=5):
    print(f"Going to location: {location}")
    vehicle.groundspeed = groundspeed
    vehicle.simple_goto(location)
    
    while True:
        current_loc = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_loc, location)
        if distance < 1.0:  
            print("Reached destination")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def setup_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('localhost', SOCKET_PORT))
    s.listen(1)
    return s

def send_mavros_command(conn, command):
    try:
        conn.send(json.dumps(command).encode())
    except Exception as e:
        print(f"Socket error: {e}")

def activate_magnet(state):
    print(f"{'Activating' if state else 'Deactivating'} electromagnet")
    return

def main():
    vehicle = connect_vehicle()
    
    sock = setup_socket()
    
    try:
        arm_and_takeoff(vehicle, CRUISE_ALTITUDE)
        
        pickup_loc = LocationGlobalRelative(*PICKUP_HEX_COORDS, CRUISE_ALTITUDE)
        goto_location(vehicle, pickup_loc)
        
        descend_loc = LocationGlobalRelative(*PICKUP_HEX_COORDS, DESCENT_ALTITUDE)
        goto_location(vehicle, descend_loc)
        
        print("Initiating computer vision positioning...")
        conn, addr = sock.accept()
        print(f"MAVROS connection from {addr}")
        
        send_mavros_command(conn, {"command": "start_cv", "target": "hexagon"})
        
        data = conn.recv(1024)
        if data:
            response = json.loads(data.decode())
            if response.get("status") == "centered":
                print("Payload centered, final descent and pickup")
                
                pickup_height = 0.5  
                goto_location(vehicle, LocationGlobalRelative(
                    response["lat"], 
                    response["lon"], 
                    pickup_height
                ))
                
                activate_magnet(True)
                time.sleep(2)  
                
                goto_location(vehicle, LocationGlobalRelative(
                    response["lat"], 
                    response["lon"], 
                    CRUISE_ALTITUDE
                ))
        
        dropoff_loc = LocationGlobalRelative(*DROPOFF_TRI_COORDS, CRUISE_ALTITUDE)
        goto_location(vehicle, dropoff_loc)
        
        descend_loc = LocationGlobalRelative(*DROPOFF_TRI_COORDS, DESCENT_ALTITUDE)
        goto_location(vehicle, descend_loc)
        
        print("Initiating computer vision positioning for dropoff...")
        send_mavros_command(conn, {"command": "start_cv", "target": "triangle"})
        
        data = conn.recv(1024)
        if data:
            response = json.loads(data.decode())
            if response.get("status") == "centered":
                print("Drop point centered, final descent")
                
                drop_height = 0.3  
                goto_location(vehicle, LocationGlobalRelative(
                    response["lat"], 
                    response["lon"], 
                    drop_height
                ))
                
                activate_magnet(False)
                time.sleep(1) 
                
                goto_location(vehicle, LocationGlobalRelative(
                    response["lat"], 
                    response["lon"], 
                    CRUISE_ALTITUDE
                ))
        
        print("Returning to launch")
        vehicle.mode = VehicleMode("RTL")
        
        while vehicle.armed:
            time.sleep(1)
        
        print("Mission complete")
        
    finally:
        vehicle.close()
        sock.close()

if __name__ == "__main__":
    main()