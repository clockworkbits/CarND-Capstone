from yaw_controller import YawController
from pid import PID
import rospy

from lowpass import LowPassFilter #TODO: find out how to use it

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

ENABLE_BRAKES = True #Experment Brakes toggle

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
        self.lowpass_tau = .03#1 is default value, it should be the max steering value allowed to avoid jerk
        self.lowpass_throttle = LowPassFilter(self.lowpass_tau, self.sample_time)
        self.pid_throttle = PID( 3.0, 0.0, 0.5, args[4], args[5] )
        self.pid_steer = PID( 2.0, 0.15, 0.6, (-1*args[3]), args[3] ) 
        #10.0, 0.01, 1.0
        #10.0, 0.0, 0.6 ,best KD 0.6 :: tested in lot also
        #10.0, 0.01, 1.0#COUNT_CTE : 1013.000000,MEAN_CTE : 0.211059,MAX_CTE : 190.695391
        #10.0, 0.01, 1.0#COUNT_CTE : 1008.000000, MEAN_CTE : 0.023891, MAX_CTE : 0.449383#waypoints values change
        #10.0, 0.0, 1.1#COUNT_CTE : 1010.000000, MEAN_CTE : 0.189719, MAX_CTE : 171.732664
        #10.0, 0.0, 0.6#COUNT_CTE : 1041.000000, MEAN_CTE : 0.016812, MAX_CTE : 0.155567
                       #COUNT_CTE : 1041.000000, MEAN_CTE : 0.018295, MAX_CTE : 0.357466
        #10.0, 0.00001, 0.6#COUNT_CTE : 1052.000000, MEAN_CTE : 0.022128, MAX_CTE : 0.191050
        #10.0, 0.0001, 0.6 #COUNT_CTE : 1046.000000, MEAN_CTE : 0.021837, MAX_CTE : 0.479986
        #10.0, 0.0, 0.6#    COUNT_CTE : 4829.000000, MEAN_CTE : 0.051810, MAX_CTE : 144.788670 @50HZ
        #10.0, 0.0, 50.0#   COUNT_CTE : 4776.000000, MEAN_CTE : 0.623201, MAX_CTE : 300.597765 @50HZ
        #10.0, 0.0, 10.0#   COUNT_CTE : 5810.000000, MEAN_CTE : 0.119992, MAX_CTE : 293.699178 @50HZ
        #10.0, 0.0, 5.0#    COUNT_CTE : 4722.000000, MEAN_CTE : 0.149059, MAX_CTE : 257.400179 @50HZ
        #10.0, 0.0, 0.1#    COUNT_CTE : 4791.000000, MEAN_CTE : 0.112050, MAX_CTE : 251.903834 @50HZ
        #10.0, 0.01, 0.1#   COUNT_CTE : 4692.000000, MEAN_CTE : 0.144895, MAX_CTE : 400.834809 @50HZ
        #10.0, 0.001, 0.6#  COUNT_CTE : 1035.000000, MEAN_CTE : 0.020627, MAX_CTE : 0.282186
        #10.0, 0.001, 0.6#  COUNT_CTE : 4750.000000, MEAN_CTE : 0.176363, MAX_CTE : 353.926827 @50HZ
        #10.0, 0.01, 0.6#   COUNT_CTE : 4785.000000, MEAN_CTE : 0.067480, MAX_CTE : 238.533552 @50HZ
        #10.0, 0.01, 0.6#   COUNT_CTE : 5221.000000, MEAN_CTE : 0.070827, MAX_CTE : 244.542952 @50HZ
        #10.0, 0.1, 0.6#    COUNT_CTE : 4764.000000, MEAN_CTE : 0.073771, MAX_CTE : 239.936489 @50HZ
        # 2.0, 0.1, 0.6#    COUNT_CTE : 4782.000000, MEAN_CTE : 0.059511, MAX_CTE : 157.302448 @50HZ - overshoots on lot
        # 2.0, 0.15,0.6#    COUNT_CTE : 5274.000000, MEAN_CTE : 0.205780, MAX_CTE : 428.199285 @50HZ - better on lot
                           #COUNT_CTE : 6399.000000, MEAN_CTE : 0.025065, MAX_CTE : 0.779938
                           #COUNT_CTE : 5416.000000, MEAN_CTE : 0.112292, MAX_CTE : 296.777104

        #10.0, 0.0, 0.5#COUNT_CTE : 1038.000000, MEAN_CTE : 0.019208, MAX_CTE : 0.332994
                       #COUNT_CTE : 1098.000000, MEAN_CTE : 0.016040, MAX_CTE : 0.242053
                       #COUNT_CTE : 1037.000000, MEAN_CTE : 0.018171, MAX_CTE : 0.263230#best cooling conditions
       
        #10.0, 0.0, 0.3#COUNT_CTE : 1063.000000, MEAN_CTE : 0.020972, MAX_CTE : 0.257886


        self.TOTAL_CTE = 0.0
        self.MEAN_CTE = 0.0
        self.MAX_CTE = 0.0
        self.COUNT_CTE = 0.0
        
        pass
    
    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        ''' args = [0:proposed_linear_velocity,
                    1:proposed_angular_velocity,
                    2:current_linear_velocity,  
                    3:current_angular_velocity
                    4:is_dbw_enabled] from DBWNode dbw_node.py
        '''
        if args[4] : #if is_dbw_enabled
            #rospy.logdebug("proposed_angular_velocity:%f",args[1])    
            #rospy.logdebug("current_angular_velocity:%f",args[3])
            #steer =  self.yaw_controller.get_steering(args[0],args[1],args[2])#bad and delayed but smooth
            steer_CTE = args[1]-args[3]
            #rospy.logdebug("steer_CTE :%f",steer_CTE)
            
            steer = self.pid_steer.step(steer_CTE, self.sample_time)
            #rospy.logdebug("steer :%f",steer)
            #steer = self.lowpass_steer.filt(steer)#TODO: test before uncomment and commit
            throttle_CTE = args[0]-args[2] #proposed_linear_velocity - current_linear_velocity
            throttle = self.pid_throttle.step(throttle_CTE,self.sample_time)#1/15 or 1/50
            brake = 0
            #Brakes system
            if ENABLE_BRAKES : 
                if throttle < 0:
                    #code refereace : https://discussions.udacity.com/t/what-is-the-range-for-the-brake-in-the-dbw-node/412339
                    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * (throttle*-1) * self.wheel_radius
                    throttle = 0
                    #rospy.logdebug("^^^^BREAK^^^^")    
                else : 
                    brake = 0
                    #rospy.logdebug("throttle = %f",throttle)
                #car used to NEVER stop on trafic lights, it was sliding sooo slow
                #to fix it :
                #hold brakes if proposed_linear_velocity too low while no brakes
                if (args[0] < 0.01) and (brake < self.brake_deadband):
                    brake = self.brake_deadband
                    rospy.logdebug("========== HOLD BRAKE ========")
            #End Brakes system
            if throttle != 0 :
                self.COUNT_CTE +=1
                self.TOTAL_CTE += abs(steer_CTE)
                self.MEAN_CTE = self.TOTAL_CTE / self.COUNT_CTE
                if self.MAX_CTE < abs(steer_CTE) : self.MAX_CTE = abs(steer_CTE)
                rospy.logdebug("COUNT_CTE : %f, MEAN_CTE : %f, MAX_CTE : %f",self.COUNT_CTE,self.MEAN_CTE,self.MAX_CTE)

        return throttle, brake, steer
        
        
        
        
        
        
        
        
        