# main.py
import time
from config import *
from sensors import LidarThread, estop_pressed
from localization import Map, ParticleFilter
from navigator import Navigator
from uv_control import uv_off
from motor_control import cleanup

def main():
    try:
        # load map (adjust resolution to your map)
        m = Map(MAP_PGM, resolution=0.01)
        pf = ParticleFilter(m)
        # initialize particles in reasonable space (set to your prototype bounds)
        pf.init_uniform(0.1, 1.8, 0.1, 1.8)

        # start lidar
        lidar = LidarThread(RPLIDAR_PORT)
        lidar.start()
        time.sleep(2)

        nav = Navigator(pf, lidar)
        # sample waypoints (x,y in meters) - set to match your map
        nav.set_waypoints([(0.3,0.3), (1.2,0.3), (1.2,1.2), (0.3,1.2)])

        last_time = time.time()
        while True:
            if estop_pressed():
                print("EMERGENCY STOP PRESSED — shutting down")
                uv_off()
                break
            scan = lidar.get_scan()
            # dummy odometry update (you should compute delta from encoders)
            # For prototype: assume small forward movement between loops
            dt = time.time() - last_time
            # Here insert real odometry deltas computed from encoders
            pf.motion_update(delta_trans=0.0, delta_rot=0.0)
            # weight particles by lidar
            if len(scan) > 10:
                pf.measure_probability(scan)
                pf.resample()
            # run navigation mission (blocking)
            nav.run_mission()
            # mission ended
            print("Mission complete")
            break

    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        uv_off()
        cleanup()

if __name__ == "__main__":
    main()
