import pid 
import math
import rospy
import yaw_controller

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_params):
        # TODO: Implement
        self.vehicle_mass, self.fuel_capacity, self.brake_deadband, self.decel_limit, self.accel_limit, self.wheel_radius, self.wheel_base, self.steer_ratio, self.max_lat_accel, self.max_steer_angle, self.min_speed = vehicle_params
        
        # init pid        
        self.pid_throttle_brake = pid.PID(1, 0.00001, 0.001, self.decel_limit, self.accel_limit)
        self.pid_steer = pid.PID(5, 0.00001, 0.001, -self.max_steer_angle, self.max_steer_angle)

	self.yaw_controller = yaw_controller.YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def get_steering(self, target_ang_v, curr_ang_v, sample_time, target_v, curr_v):
	# feedback PID
        error_ang_v = target_ang_v.z - curr_ang_v.z
        steering = self.pid_steer.step(error_ang_v, sample_time)
        # feedforward Yaw Controller
	yaw_control = self.yaw_controller.get_steering(target_v.x, target_ang_v.z, curr_v.x)
	# feedforward + feedback
	steer = 1*steering + 1*yaw_control

        return steer
        

    def get_throttle_brake(self, target_v, curr_v, sample_time):
	
        error_v = math.sqrt(target_v.x**2 + target_v.y**2) - math.sqrt(curr_v.x**2 + curr_v.y**2)
        throttle = self.pid_throttle_brake.step(error_v, sample_time)
        brake = 0
        if throttle < 0 :
            brake = -throttle*5
            throttle = 0
        
        return throttle,brake 

    def control(self, target_v, curr_v, target_ang_v, curr_ang_v,dbw_enabled, sample_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.0
        brake = 0.0
        steering = 0.0
        #target_v.x = 20 #target speed
        if dbw_enabled:
            throttle, brake  = self.get_throttle_brake(target_v, curr_v, sample_time)
            steering = self.get_steering(target_ang_v, curr_ang_v, sample_time, target_v, curr_v)
            
            if brake < 5 and target_v.x < 1:
                brake = 5
            if curr_v.x <= 1 and target_v.x <= 1:
                brake = 1
            #rospy.loginfo("throttle - {}  brake - {} ".format(throttle, brake))
            #rospy.loginfo("target_v - {}  curr_v - {} ".format(target_v, curr_v))
        else:
            pass
        return throttle, brake, steering
