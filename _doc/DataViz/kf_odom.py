import numpy as np



dt_vescStateCallback = 1/50 # time passing between each sample

class Odom:
    def __init__(self):
        # Conversion params (from vesc.yaml)
        self.speed_to_erpm_offset = 0
        self.speed_to_erpm_gain = 4140.0

        self.steering_angle_to_servo_offset = 0.525
        self.steering_angle_to_servo_gain = -0.95

        self.wheelbase = 0.315

    def getSpeed(self, rpm): 
        # NOTE: In vesc_to_odom.cpp, rpm is called 'state.speed' and speed is called 'current_speed'
        return (rpm - self.speed_to_erpm_offset) / self.speed_to_erpm_gain
         
    
    def getSteeringAngle(self, servo):
        # 'current_steering_angle'
        return (servo - self.steering_angle_to_servo_offset) / self.steering_angle_to_servo_gain
    
    def getAngularVelocity(self, speed, steeringAngle):
        return speed * np.tan(steeringAngle) / self.wheelbase


    def run(self, servo, rpm, theta_imu_dot, dt=0.02):
        """ 
            Returns odom outputs: pos, orient, pos_dot, orient_dot
        """

        # Convert hardware signals into physical units
        speed = self.getSpeed(rpm)
        
        # calc state of theta_dot and theta
        theta_dot = self.getAngularVelocity(speed, self.getSteeringAngle(servo))
        
        #TODO: fuse odom theta w/ imu theta here
        # rudimentary "sensor fusion": use IMU info instead of odom info
        theta_dot = theta_imu_dot

        theta = np.cumsum(theta_dot * dt_vescStateCallback)

        # calc state of pos_dot and pos
        x_dot = speed * np.cos(theta)
        y_dot = speed * np.sin(theta)
        pos = np.hstack((np.cumsum(x_dot*dt)[:,np.newaxis], 
                         np.cumsum(y_dot*dt)[:,np.newaxis]))

        pos_dot = np.hstack((x_dot[:,np.newaxis], y_dot[:,np.newaxis]))
        state = pos, theta, pos_dot, theta_dot
        
        return state


