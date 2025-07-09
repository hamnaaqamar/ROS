import socket
import random
import time
import json

def simulate_yolo_detection():
    """Generates fake normalized coordinates that gradually center"""
    x, y = random.random(), random.random()
    x = 0.5 + (x - 0.5) * 0.9  
    y = 0.5 + (y - 0.5) * 0.9
    return x, y

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', 5000))
    
    while True:
        # Simulate receiving activation command
        try:
            data = sock.recv(1024, socket.MSG_DONTWAIT)
            if data and b"start_cv" in data:
                print("YOLO simulator activated")
                for _ in range(20):  # Send 20 simulated detections
                    x, y = simulate_yolo_detection()
                    msg = {
                        "header": {"stamp": {"secs": 0, "nsecs": 0}},
                        "pose": {"position": {"x": x, "y": y}}
                    }
                    sock.send(json.dumps(msg).encode())
                    time.sleep(0.1)
                
                # Send centered signal
                sock.send(json.dumps({"status": "centered"}).encode())
        except BlockingIOError:
            pass
        
        time.sleep(0.1)

if __name__ == "__main__":
    main()