from pymavlink import mavutil
import time

class DronePymavlink:
    def __init__(self, connection_string="udp:127.0.0.1:14551"):
        print("[INFO] Connecting to SITL...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat(timeout=30)
        print(f"[INFO] Connected to system (system {self.master.target_system} component {self.master.target_component})")

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"[INFO] Mode set to {mode}")

    def arm(self):
        print("[INFO] Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        self.master.motors_armed_wait()
        print("[INFO] Armed")

    def takeoff(self, altitude=10):
        print(f"[INFO] Taking off to {altitude} meters...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, altitude
        )
        time.sleep(1)
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_alt = msg.relative_alt / 1000.0  # mm -> meters
            if current_alt >= altitude :
                print(f"[INFO] Reached altitude: {current_alt:.1f} m")
                break

    def yaw_relative(self, angle=90):
        print(f"[INFO] Rotating yaw by {angle}°...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle,  # target angle
            20,     # yaw speed deg/s
            1,      # direction 1=cw, -1=ccw
            1,      # relative (1) or absolute (0)
            0, 0, 0
        )
        time.sleep(5)
        print("[INFO] Rotation complete")

    def land(self):
        print("[INFO] Landing...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )
        self.master.motors_disarmed_wait()
        print("[INFO] Landed")
    
    def close(self):
        self.master.close()


if __name__ == "__main__":
    drone = DronePymavlink()

    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(10)
    drone.yaw_relative(90)
    drone.set_mode("LAND")
    drone.land()

    drone.close()