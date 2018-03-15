from yaw_controller import YawController
from pid import PID
import rospy
#import math

from lowpass import LowPassFilter #TODO: find out how to use it

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        '''args = [0:wheel_base,    1:steer_ratio,
                   2:max_lat_accel, 3:max_steer_angle,
                   4:decel_limit,   5:accel_limit
                   6:sample_rate
        ]'''
        self.steer_ratio = args[1]##no need
        self.min_speed = (rospy.get_param('/waypoint_loader/velocity') * 1000.) / (60. * 60.) #kmph to mps
        self.yaw_controller = YawController(args[0],#wheel_base
                                            args[1],#steer_ratio
                                            self.min_speed ,
                                            args[2],#max_lat_accel
                                            args[3])#max_steer_angle
        self.sample_time = 1.0/args[6] # 1/sample_rate
        self.lowpass_tau = 1#1 is default value, it should be the max steering value allowed to avoid jerk
        self.lowpass_steer = LowPassFilter(1, self.sample_time)
        self.pid_throttle = PID( 3.0, 0.0, 0.5, args[4], args[5] )
        '''
        ### DEBUGGING        
        self.last_cte = 0        
        self.count = 0        
        self.err_all = 0        
        self.err_avg = 0
        self.err_min = 1000
        self.err_max = 0
        ### END OF DEBUGGING
        '''
        #0.12, 0.0, 3 , from my pid project is not working
        
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        ''' args = [0:proposed_linear_velocity,
                    1:proposed_angular_velocity,
                    2:current_linear_velocity,  
                    3:current_angular_velocity,
                    4:brake_deadband
                    5:is_dbw_enabled] from DBWNode dbw_node.py
        '''
        if args[5] : #if is_dbw_enabled
            brake = 0
            steer =  self.yaw_controller.get_steering(args[0],args[1],args[2])#bad and delayed but smooth
            #steer = self.lowpass_steer.filt(steer)#TODO: test before uncomment and commit
            throttle_CTE = args[0]-args[2] #proposed_linear_velocity - current_linear_velocity
            
            throttle = self.pid_throttle.step(throttle_CTE,self.sample_time)#1/15 or 1/50
            if throttle < args[4]*-1:
                rospy.logdebug("^^^^BREAK^^^^")    
                brake = -throttle
                throttle = 0
            else : 
                rospy.logdebug("throttle = %f",throttle)
                breke = 0
            #car used to NEVER stop on trafic lights, it was sliding sooo slow
            #to fix it :
            #hold brakes if proposed_linear_velocity too low while no brakes
            if (args[0] < 0.01) and (brake < args[4]):
                rospy.logdebug("HOLD BRAKE")
                brake = args[4]
            #brake = 
            # Return throttle, brake, steer
            '''
            ### DEBUGING            
            if self.last_cte != throttle_CTE and throttle_CTE < 0.3 and throttle_CTE != 0.0:
                self.last_cte = throttle_CTE 
                self.count += 1
                self.err_all += abs(throttle_CTE)
                self.err_avg = self.err_all/self.count
                if self.err_max < abs(throttle_CTE):
                    self.err_max = abs(throttle_CTE)
                if self.err_min > abs(throttle_CTE):
                    self.err_min = abs(throttle_CTE)
            rospy.logdebug("throttle_CTE =%f , avg=%f, min=%f, max=%f",throttle_CTE , self.err_avg, self.err_min, self.err_max)    
            #END OF DEBUGING
            '''
        #else return default values        
        #return 1., 0., 0.2
        return throttle, brake, steer