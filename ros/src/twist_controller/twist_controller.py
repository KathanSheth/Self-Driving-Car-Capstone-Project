import rospy
import math
from   yaw_controller import YawController
from   pid            import PID
from   lowpass        import LowPassFilter
from std_msgs.msg import Float32

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        Kp = 0.3
        Ki = 0.1
        Kd = 0.0
        mn = 0.0
        mx = 0.2
        self.throttle_controller = PID(Kp, Ki, Kd, mn, mx)

        tau=0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        # Use a low pass filter to dampen the steering
        steer_tau = 0.2
        steer_ts = 1.0
        self.steer_lpf = LowPassFilter(steer_tau, steer_ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.steering_controller = PID(kp=0.5, ki=0.004, kd=0.25, mn=-max_steer_angle, mx=max_steer_angle)
        self.last_time = rospy.get_time()  

    def control(self, cur_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        rospy.loginfo('dbw %s Vel %s',dbw_enabled,cur_vel)
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.steering_controller.reset()
            return 0.,0.,0.
        cur_vel = self.vel_lpf.filt(cur_vel)
        yaw_steering = self.yaw_controller.get_steering(linear_vel, angular_vel, cur_vel)

        vel_err = linear_vel - cur_vel
        self.last_vel = cur_vel
        
        cur_time = rospy.get_time()
        sample_time = cur_time - self.last_time
        self.last_time = cur_time

        throttle = self.throttle_controller.step(vel_err, sample_time)
        pid_steering = self.steering_controller.step(angular_vel, sample_time)
        steering = pid_steering + 1.0 * yaw_steering

        # Apply Low Pass Filter to steering to dampen it.
        steering = self.steer_lpf.filt(steering)
        brake = 0

        if linear_vel == 0. and cur_vel < 0.1:
            throttle = 0
            brake = 700
        elif throttle <0.1 and vel_err <0:
            throttle = 0
            decel = max(vel_err, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius

        return throttle, brake, steering
