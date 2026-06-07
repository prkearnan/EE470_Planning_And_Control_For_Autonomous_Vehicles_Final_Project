import math
import time
import sys
import carla
import numpy as np

import matplotlib.pyplot as plt
import math


sys.path.append("C:/Users/jkear/Documents/CARLA_0.9.16/PythonAPI/carla")
from agents.navigation.global_route_planner import GlobalRoutePlanner


HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0 #wait for server connection

MAP_NAME = "Town03" #need to pick a map
VEHICLE_BP = "vehicle.tesla.model3"

SPAWN_INDEX = 0 
TARGET_SPEED = 20 / 3.6 #20 km/h converted to m/s (TUNE)

WAYPOINT_DISTANCE = 1.5 # meters between sampled lane ceneterd waypoints (TUNE)
LOOKAHEAD_COUNT = 50 #how many waypoints to pre-fetch ahead of the car
GOAL_WAYPOINT = 24 # (9 - 91 "hook") (0 - 24 "half roundabout") (46 - 52 "Hills")

STATE = "PP"   # choose either "PID" or "PP"

 
FIXED_DELTA_T = 0.05    # seconds — synchronous simulation timestep (20 Hz)
RUN_DURATION = 60.0    # seconds to run before auto-exit (0 = run forever)

def connect_to_carla(host: str, port: int, timeout: float) -> carla.Client:
    """
        Connect to CARLA server and return the client
    """

    print(f"[CARLA] Connecting to {host}:{port} ...")
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    try:
        server_version = client.get_server_version()
        print(f"[CARLA] Connected. Server Version: {server_version} ")
    except RuntimeError as e:
        sys.exit(f"[ERROR] Coult Not Connect to CARLA: {e}")

    return client

def load_map(client:carla.Client, map_name: str) -> carla.World:
    """
        Load the requested map
    """

    world = client.get_world()
    current_map = world.get_map().name.split("/")[-1]
    if current_map != map_name:
        print(f"[CARLA] Loading Map {map_name} (was) {current_map}")
        world = client.load_world(map_name)
        time.sleep(2.0) #gives the server a moment to load 
    else:
        print(f"[CARLC] Map {map_name} already loaded.") 
    return world  

def set_synchronous_mode(world: carla.World, delta_t: float) -> carla.WorldSettings:
    """
        Enables sync mode so our script drives the simulation clock
        returns og settings to restore after 
    """
    original_settings = world.get_settings()
    settings = world.get_settings() # settings = original_settings would make both vartiable point to the same object, but we need seperate ones 
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = delta_t
    world.apply_settings(settings)
    print(f"[CARLA] Synchronous mode ON (Δt = {delta_t}s, {1/delta_t: .0f} Hz)")
    return original_settings

def spawn_vehicle(world:carla.World, blueprint_id: str, spawn_index: int) -> carla.Vehicle:
    """
        Spawn the vehicle at the chosen spawn point
    """

    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find(blueprint_id)
    if vehicle_bp is None:
        sys.exit(f"[ERROR] Blueprint '{blueprint_id}' not found.")
    
    spawn_points = world.get_map().get_spawn_points()
    if spawn_index >= len(spawn_points):
        sys.exit(f"[ERROR] Spawn index {spawn_index} out of range (max {len(spawn_points)-1}).")
    
    transform = spawn_points[spawn_index]
    vehicle = world.spawn_actor(vehicle_bp, transform)
    print(f"[CARLA] Spawned '{blueprint_id}' at spawn point {spawn_index}.")
    return vehicle

def get_waypoints_ahead(world: carla.World, vehicle: carla.Vehicle, spacing: float, count: int) -> list[carla.Waypoint]:
    """
        Samples lane-centered waypoints: "count many", "spacing spaced"
        Referecene path for both controllers
    """
    carla_map = world.get_map()
    start_wp = carla_map.get_waypoint( vehicle.get_location(), 
                                       project_to_road = True, 
                                       lane_type = carla.LaneType.Driving 
    )

    waypoints = [start_wp]
    current = start_wp
    for _ in range(count -1):
        nexts = current.next(spacing)
        if not nexts:
            break
        current = nexts[0] #follow the road
        waypoints.append(current)

    return waypoints

def attach_spectator(world: carla.World, vehicle: carla.Vehicle) -> None:
    """
        Position the spectator camera above-behind the vehicle.
    """
    transform = vehicle.get_transform()
    # Get a point 10m behind and 5m above in the car's local frame
    backward = transform.get_right_vector() * 0  
    location = transform.location - transform.get_forward_vector() * 10 + carla.Location(z=5)
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        location,
        carla.Rotation(pitch=-20, yaw=transform.rotation.yaw)
    )) 

def lateral_error(vehicle: carla.Vehicle, waypoint: carla.Waypoint) -> float:
    """
        signed lateral distance from the vehicle to the waypoint
        positive = vehicle is to right 
        both controllers will use error
    """    
    vehicle_loc = vehicle.get_location()
    wp_loc = waypoint.transform.location

    #vector from waypoint to vehicle
    dx = vehicle_loc.x - wp_loc.x
    dy = vehicle_loc.y - wp_loc.y

    #waypoint forward direciton
    yaw_rad = math.radians(waypoint.transform.rotation.yaw) #carla stores in degress
    fwd_x = math.cos(yaw_rad)
    fwd_y = math.sin(yaw_rad)

    #cross product gives signed lateral offset
    error = -(dx * (-fwd_y) + dy * fwd_x)

    return error


def get_route_waypoints(world, vehicle, spacing=2.0):
    carla_map = world.get_map()
    
    # pick a destination spawn point far away
    spawn_points = carla_map.get_spawn_points()
    destination = spawn_points[GOAL_WAYPOINT].location  # change index to pick different destinations
    
    # use the global route planner
    grp = GlobalRoutePlanner(carla_map, sampling_resolution=spacing)
    
    route = grp.trace_route(vehicle.get_location(), destination)
    
    # route is a list of (waypoint, road_option) tuples
    waypoints = [wp for wp, _ in route]

    return waypoints


############################################

def get_rear_axle_position_2d(vehicle):
    t = vehicle.get_transform()
    # Rear axle is L/2 = 1.4375m behind the center
    yaw_rad = np.radians(t.rotation.yaw)
    rear_x = t.location.x - 1.4375 * np.cos(yaw_rad)
    rear_y = t.location.y - 1.4375 * np.sin(yaw_rad)
    return np.array([rear_x, rear_y])


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class PID:
    def __init__(self, Kp, Kd, Ki, ):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.integral = 0.0
        self.prev_error  = 0.0

    def updatePID(self, error, delta_t):
        derivative = (error - self.prev_error)/delta_t
        self.integral += error * delta_t

        self.prev_error = error
        steering = self.kp * error + self.ki * self.integral + self.kd * derivative 
        return max(-1.0, min(1.0, steering))    

class PurePursuit:
    def __init__(self, waypoints, k=0.05):
        self.waypoints = waypoints
        self.wp_index = 0  
        self.k = k # lookahead gain (tune this)
        self.L = 2.875 # Tesla Model 3 wheelbase (m)
        self.min_Ld = 4.5 # minimum lookahead distance (m)
        self.max_steering = np.radians(70) # Tesla max steering angle

    def update(self, vehicle, wp_index):
        rear_pos = get_rear_axle_position_2d(vehicle)
        yaw_raw = np.radians(vehicle.get_transform().rotation.yaw)

        vel = vehicle.get_velocity()
        speed = np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        Ld = max(self.min_Ld, self.k * speed)

        # search forward from main loop's current wp_index
        search_end = min(wp_index + 30, len(self.waypoints))  # hard cap on lookahead search

        target = None
        for i in range(wp_index, search_end):
            wp_loc = np.array([self.waypoints[i].transform.location.x,
                       self.waypoints[i].transform.location.y])
            dist = np.linalg.norm(wp_loc - rear_pos)
            to_wp = wp_loc - rear_pos
            fwd = np.array([np.cos(yaw_raw), np.sin(yaw_raw)])
            if dist >= Ld and np.dot(fwd, to_wp) > 0:
                target = wp_loc
                break

# If nothing found in window, just use the nearest waypoint in window
        if target is None:
            target = np.array([self.waypoints[min(wp_index + 1, len(self.waypoints)-1)].transform.location.x,
                       self.waypoints[min(wp_index + 1, len(self.waypoints)-1)].transform.location.y])

        dx = target[0] - rear_pos[0]
        dy = target[1] - rear_pos[1]
        print(f"rear={rear_pos} | target={target} | dx={dx:.2f} dy={dy:.2f} | yaw_deg={np.degrees(yaw_raw):.1f}")   

        dy = target[1] - rear_pos[1]
        dx = target[0] - rear_pos[0]

        alpha_wrap = wrap_to_pi(np.arctan2(dy, dx) - yaw_raw)
        delta = np.arctan2(2 * self.L * np.sin(alpha_wrap), Ld)
        steering = float(np.clip(delta / self.max_steering, -1.0, 1.0))
        print(f"alpha={np.degrees(alpha_wrap):.1f} | Ld={Ld:.2f} | wp_index={wp_index} | steering={steering:.3f} | arctan(dy/dx)={np.degrees(np.arctan2(dy, dx)):.1f}")
        return steering

#***************************MAIN***************************************************************

def main():
    client = connect_to_carla(HOST, PORT, TIMEOUT)
    world = load_map(client, MAP_NAME)
    orig_cfg = set_synchronous_mode(world, FIXED_DELTA_T)

    vehicle = None

    try:
        vehicle = spawn_vehicle(world, VEHICLE_BP, SPAWN_INDEX)
        world.tick() #one tick for physics

        # DEBUG 
        vehicle_loc = vehicle.get_location()
        print(f"Vehicle: ({vehicle_loc.x:.1f}, {vehicle_loc.y:.1f})")
        print(f"Vehicle yaw: {vehicle.get_transform().rotation.yaw:.1f}")

        #pre fetch the reference path
        waypoints = get_route_waypoints(world, vehicle, spacing=WAYPOINT_DISTANCE)
        #waypoints = get_waypoints_ahead(world, vehicle, WAYPOINT_DISTANCE, 300)

        # DEBUG 
        wp0 = waypoints[0].transform.location
        print(f"WP0:     ({wp0.x:.1f}, {wp0.y:.1f})")
        print(f"WP0 yaw:     {waypoints[0].transform.rotation.yaw:.1f}")

        for i in range(min(10, len(waypoints))):
            wp = waypoints[i]
            start = wp.transform.location + carla.Location(z=0.5)
            yaw = math.radians(wp.transform.rotation.yaw)
            end = start + carla.Location(x=math.cos(yaw)*2, y=math.sin(yaw)*2)
            world.debug.draw_arrow(start, end, thickness=0.1, 
                           color=carla.Color(255, 0, 0), life_time=30.0)
        

        path_x = []
        path_y = []

        for wp in waypoints:
            loc = wp.transform.location
            path_x.append(-loc.x)
            path_y.append(loc.y)
        ####

        pure_pursuit = PurePursuit(waypoints, k=0.05) #
        pid_lateral = PID(Kp = 0.8, Ki = 0.01, Kd = 0.01) #balacnced choice Kp=0.8, Ki=0.01, Kd=0.01 ; Better control, less comfort Kp=1.2, Ki=0.01, Kd=0.01 ; opposite Kp= 0.6 etc.

        print(f"[Path] {len(waypoints)} waypoints loaded"
              f"({WAYPOINT_DISTANCE * len(waypoints):.1F} m of road ahead)")
        
        #draw reference path in carla
        debug = world.debug
        for wp in waypoints:
            debug.draw_point(
                wp.transform.location + carla.Location(z=0.3),
                size = 0.1, color = carla.Color(0, 255, 0), life_time = 10000.0
            )

        
        print("\n[Loop] Running Control Loop. Press Ctrl +C to stop. \n")
        start_time = time.time()
        wp_index = 0 #index of nearest upcoming waypoint

        #PLotting vector
        speed_times = []
        speed_values = []

        actual_x = []
        actual_y = []

        lat_err_times = []
        lat_err_values = []
        ####

        steer_times = []
        steer_values = []
        steer_jerk = []  # rate of change of steering

        prev_steer = 0.0

        while True:
            elapsed = time.time() - start_time
            if RUN_DURATION > 0 and elapsed > RUN_DURATION:
                print(f"[Loop] {RUN_DURATION}s elapsed - exiting.")
                break

            #update spectator camera
            attach_spectator(world, vehicle)

            #find cloests upcoming waypoint
            vehicle_loc = vehicle.get_location()
            
            actual_x.append(-vehicle_loc.x)
            actual_y.append(vehicle_loc.y)
            ####

            min_dist = float('inf')
            nearest_idx = wp_index
            for i in range(wp_index, min(wp_index + 20, len(waypoints))):
                d = vehicle_loc.distance(waypoints[i].transform.location)
                if d < min_dist:
                    min_dist = d
                    nearest_idx = i

            wp_index = nearest_idx

            target_wp = waypoints[wp_index]
            lat_err = lateral_error(vehicle, target_wp)

            lat_err_times.append(elapsed)
            lat_err_values.append(lat_err)
            ###
            if wp_index >= len(waypoints) - 5:
                print("[Loop] Destination reached.")
                break

            #place holder contros
            control = carla.VehicleControl()
            control.throttle = 0.3
            

            if STATE == "PID":
                steering = pid_lateral.updatePID(lat_err, FIXED_DELTA_T)

            elif STATE == "PP":
                steering = pure_pursuit.update(vehicle, wp_index)
            ###
                        
            control.steer = steering
            control.brake = 0.0
            vehicle.apply_control(control)

            #console telemetry
            vel = vehicle.get_velocity()
            speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) #m/s

            speed_times.append(elapsed)
            speed_values.append(speed)

            steer_delta = abs(steering - prev_steer) / FIXED_DELTA_T
            steer_jerk.append(steer_delta)
            steer_values.append(steering)
            steer_times.append(elapsed)
            prev_steer = steering

            ####
            

            print(f"t={elapsed:5.1f}s  |  speed={speed*3.6:5.1f} km/h  " 
                f"|  lat_err = {lat_err:+.3f} m  |  wp ={wp_index}/{len(waypoints)} "
                f"| dist={vehicle_loc.distance(waypoints[wp_index].transform.location):.2f}m")
            

            ########################

            world.tick() #advance sim by 1 time step


        print(f"Mean steering jerk: {np.mean(steer_jerk):.4f} rad/s")
        print(f"Max steering jerk:  {np.max(steer_jerk):.4f} rad/s")
        print(f"Total steering jerk: {np.sum(steer_jerk):.4f} rad")
        
        plt.figure()
        plt.plot(speed_times, speed_values)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")
        plt.title("Vehicle Speed Over Time")
        plt.grid(True)
       
        plt.figure()
        plt.plot(path_x, path_y, label="Intended Path")
        plt.plot(actual_x, actual_y, label="Actual Vehicle Path")

        plt.scatter(path_x[0], path_y[0], s=120, marker="o", label="Start")
        plt.scatter(path_x[-1], path_y[-1], s=150, marker="*", label="Goal")

        plt.xlabel("X position [m]")
        plt.ylabel("Y position [m]")
        if STATE == "PID":
            plt.title("Intended Path vs Actual Vehicle Path (PID Controller)")
        elif STATE == "PP":
            plt.title("Intended Path vs Actual Vehicle Path (Pure Pursuit Controller)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        

        plt.figure()
        plt.plot(lat_err_times, lat_err_values)
        plt.xlabel("Time [s]")
        plt.ylabel("Lateral Error [m]")
        plt.title("Lateral Error Over Time")
        if len(lat_err_values) > 0:
            total_abs_lat_err = sum(abs(e) for e in lat_err_values)
            mean_abs_lat_err = total_abs_lat_err / len(lat_err_values)

            results_text = (
                f"Total abs error = {total_abs_lat_err:.3f} m\n"
                f"Mean abs error = {mean_abs_lat_err:.3f} m"
            )

            steering_text = (
                f"Total steering jerk = {np.sum(steer_jerk):.4f} rad\n"
                f"Mean steering jerk = {np.mean(steer_jerk):.4f} rad/s\n"
                f"Max steering jerk = {np.max(steer_jerk):.4f} rad/s"

            )

            plt.text(
                0.02, 0.95,
                results_text,
                transform=plt.gca().transAxes,
                verticalalignment="top",
                bbox=dict(facecolor="white", alpha=0.8)
            )

            plt.text(
                0.02, 0.05,
                steering_text,
                transform=plt.gca().transAxes,
                verticalalignment="bottom",
                bbox=dict(facecolor="white", alpha=0.8)
            )

            plt.grid(True)
                
            plt.show()
        #######
    except KeyboardInterrupt:
        print("\n[Loop] Interrupted by User.")

    finally:
        #clean up
        if vehicle is not None:
            vehicle.destroy()
            print("[CARLA] Vehilce Destroyed.")
        world.apply_settings(orig_cfg)
        print("[CARLA] Original world settings restored.")

if __name__ == "__main__":
    main()
