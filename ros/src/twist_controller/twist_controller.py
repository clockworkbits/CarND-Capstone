from yaw_controller import YawController
from pid import PID
import rospy

from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        '''args = [0:wheel_base,    1:steer_ratio,
                   2:max_lat_accel, 3:max_steer_angle,
                   4:decel_limit,   5:accel_limit
        ]'''
        self.steer_ratio = args[1]##no need
        self.min_speed = (rospy.get_param('/waypoint_loader/velocity') * 1000.) / (60. * 60.) #kmph to mps
        self.yaw_controller = YawController(args[0],#wheel_base
                                            args[1],#steer_ratio
                                            self.min_speed ,
                                            args[2],#max_lat_accel
                                            args[3])#max_steer_angle
        self.pid = PID( 0.1,0.01,1.0,args[4],args[5])
        #2.0, 0.4, 0.1
        #0.5,0.01,0.2
        #0.12, 0.0, 3
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        ''' args = [0:proposed_linear_velocity, 1:proposed_angular_velocity,
                    2:current_linear_velocity,  3:is_dbw_enabled] from DBWNode dbw_node.py
        '''
        if args[3] : #if is_dbw_enabled
            steer =  self.yaw_controller.get_steering(args[0],args[1],args[2])#bad and delayed but smooth
            #args[1] * self.steer_ratio ## bad
            #steer = 2.0*self.steer_filter.filt(steer)
            #get_steering args[1]#proposed_angular_velocity
            error = args[0] - args[2] #proposed_linear_velocity - current_linear_velocity
            throttle = 0.2#self.pid.step(error,0.02)
            #brake = 
            # Return throttle, brake, steer

        #else return default values        
        #return 1., 0., 0.2
        return throttle, 0., steer