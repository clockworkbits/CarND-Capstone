import rospy
import numpy as np
import matplotlib.pyplot as plt
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter #TODO: find out how to use it

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

USE_YAW_CONTOLLER = True # Use the Yaw controller if True, otherwise use the PID controller

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        '''args = [0:wheel_base,    1:steer_ratio,
                   2:max_lat_accel, 3:max_steer_angle,
                   4:decel_limit,   5:accel_limit
                   6:brake_deadband,7:vehicle_mass ,
                   8:fuel_capacity, 9:wheel_radius ,
                   10:sample_rate
        ]'''
        self.brake_deadband = args[6]
        self.vehicle_mass   = args[7]
        self.fuel_capacity  = args[8]
        self.wheel_radius = args[9]

        self.min_speed = (rospy.get_param('/waypoint_loader/velocity') * 1000.) / (60. * 60.) #kmph to mps
        self.yaw_controller = YawController(args[0],#wheel_base
                                            args[1],#steer_ratio
                                            0,
                                            args[2],#max_lat_accel
                                            args[3])#max_steer_angle
        self.sample_time = 1.0/args[6] # 1/sample_rate
        self.pid_throttle = PID( 1.0, 0.0, 0.5, args[4], args[5] )
        self.pid_steer = PID( 2.0, 0.15, 0.7, (-1*args[3]), args[3] ) 
        #throttle lowpass filter        
        self.lowpass_filter = LowPassFilter(500,self.sample_time)
        
        #debug
        self.debug_throttle_arr = np.array([]) 
        self.debug_throttle_err_arr = np.array([]) 
        self.debug_steering = np.array([]) 
        self.debug_proangvel = np.array([])         
        self.debug_can_debug = False
        np.set_printoptions(threshold=np.nan)#force print all array elements
        
        pass
    
    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        ''' args = [0:proposed_linear_velocity,
                    1:proposed_angular_velocity,
                    2:current_linear_velocity,  
                    3:current_angular_velocity
                    4:is_dbw_enabled] from DBWNode dbw_node.py
        '''
        throttle, brake, steer = 0,0,0
        if args[4] : #if is_dbw_enabled
            if USE_YAW_CONTOLLER:
                steer = self.yaw_controller.get_steering(args[0],args[1],args[2])#bad and delayed but smooth
            else:
                steer_CTE = args[1]-args[3]
                steer = self.pid_steer.step(steer_CTE, self.sample_time)
            
            throttle_CTE = args[0]-args[2] #proposed_linear_velocity - current_linear_velocity
            throttle = self.pid_throttle.step(throttle_CTE,self.sample_time)#1/15 or 1/50
            throttle = self.lowpass_filter.filt(throttle)
            brake = 0
            
            if self.debug_can_debug:
                self.debug_throttle_arr = np.append(self.debug_throttle_arr, throttle)
                self.debug_throttle_err_arr = np.append(self.debug_throttle_err_arr, throttle_CTE)
                self.debug_steering = np.append(self.debug_steering, steer)
                self.debug_proangvel = np.append(self.debug_proangvel, args[1])
            
            #Brakes system
            if throttle < 0:
                #code refereace : https://discussions.udacity.com/t/what-is-the-range-for-the-brake-in-the-dbw-node/412339
                brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * (throttle*-1) * self.wheel_radius
                throttle = 0
            else : 
                brake = 0
                if throttle > 0:#my debug
                    self.debug_can_debug = True#my debug
            #car used to NEVER stop on trafic lights, it was sliding sooo slow, to fix it :
            #hold brakes if proposed_linear_velocity too low while no brakes
            if (args[0] < 0.01) and (brake < self.brake_deadband):
                brake = self.brake_deadband
                self.draw_graph()#my debug
            
            #my debug
            if args[0] < 0.01: self.draw_graph()
        else :
            #Submission checklist and requirements
            self.pid_steer.reset()
            self.pid_throttle.reset()

        return throttle, brake, steer
        
        
    #my debug
    def draw_graph(self):
        #time.sleep(3)
        if self.debug_can_debug :
            self.debug_can_debug = False
            rospy.logdebug("#__ CAR STOPPPPPPPPPPPPPPPPPPPPP ")
            rospy.logdebug("#__ MEAN_CTE: %f",np.mean(self.debug_throttle_arr))            
            rospy.logdebug("#__ MEAN: %f",np.mean(self.debug_throttle_err_arr))
            rospy.logdebug("#__ thr:")
            rospy.logdebug(self.debug_throttle_arr)
            rospy.logdebug("#__ cte: ")
            rospy.logdebug(self.debug_throttle_err_arr)            
            rospy.logdebug("#__ str:")
            rospy.logdebug(self.debug_steering)
            rospy.logdebug("#__ ang: ")
            rospy.logdebug(self.debug_proangvel)            

            self.debug_throttle_arr = np.array([]) 
            self.debug_throttle_err_arr = np.array([]) 
            self.debug_steering = np.array([]) 
            self.debug_proangvel = np.array([]) 
            
        
