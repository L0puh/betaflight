#!/usr/bin/env python3
import time
from pymavlink import mavutil


t = int(input("HOW MANY SET_ATTITUDE_TARGET packets?\n>"))
DEVICE = '/dev/ttyUSB0'  
BAUD = 115200
DURATION = 10 

def main():
    print(f"Connecting to {DEVICE} at {BAUD} baud...")
    
    try:
        conn = mavutil.mavlink_connection(DEVICE, baud=BAUD)
        print("Waiting for heartbeat...")
        conn.wait_heartbeat(timeout=10)

        print(f"Heartbeat received from system {conn.target_system}")
        time_boot_ms = int(time.time())
        
        print("\n--- Sending SET_ATTITUDE_TARGET ---")
        for i in range(1, t+1):
            print(f"\t\t{i}. SENDING...")
            conn.mav.set_attitude_target_send(
                time_boot_ms=time_boot_ms,
                target_system=conn.target_system,
                target_component=1,
                type_mask=0b00000111,  
                q=[1.0, 0.0, 0.0, 0.0],  
                body_roll_rate=0.0,
                body_pitch_rate=0.0,
                body_yaw_rate=0.0,
                thrust=0.5  
            )
        print("SET_ATTITUDE_TARGET sent successfully!")
        
        print(f"\n--- Listening for messages for {DURATION} seconds ---")
        print("Press Ctrl+C to stop early\n")
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < DURATION:
            try:
                msg = conn.recv_match(blocking=True, timeout=0.5)
                
                if msg:
                    message_count += 1
                    print(f"[{message_count}] {msg.get_type()}: {msg}")
                    
            except KeyboardInterrupt:
                print("\nStopped by user")
                break
            except Exception as e:
                print(f"Error receiving message: {e}")
        
        print(f"\nReceived {message_count} messages in {DURATION} seconds")
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
