class PID():

    def __init__(self,kp=1, ki=1, kd=1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.current_error = 0
        self.prev_error = 0
        self.current_value = 0
        self.prev_value = 0
        self.total_error = 0

    def clear(self):
        self.total_error = 0
        self.prev_error = 0
        self.prev_value = 0

    def thres(self,value,limit):
        limit = abs(limit)
        if value >= limit:
            return limit
        elif value <= -limit:
            return -limit
        else:
            return value

    def get(self,value,override=False,verbose=False):
        # Value Error
        self.current_value = value 
        self.current_error =  self.current_value - self.prev_value

        # Total Error
        self.total_error = self.current_error + self.total_error

        # Derivative Error
        self.diff = self.current_error - self.prev_error

        # Threshold Errors
        self.current_error = self.thres(self.current_error,30)
        self.diff = self.thres(self.diff,30)
#        self.total_error = self.thres(self.total_error,40)
        
        # Calc PID Return
        ret = self.kp*self.current_error + self.ki*self.total_error + self.kd*self.diff
        # Threshod result angle
        ret = self.thres(ret,30)
        #ret = ret if (ret<=30) else 30
        #ret = ret if (ret>=-30) else -30

        if override: # If we want to override the previous errors with the actual angle sent to car
            # Recalc prev error
            self.prev_value = 0#ret
            self.prev_error = self.current_value #- ret
            
            # Threshold prev error
            self.prev_error = self.thres(self.prev_error,30)
            
            # Recalc total error
            #self.total_error = self.total_error - self.current_error + self.prev_error

            # Threshold total error
            self.total_error = self.thres(self.total_error,800)
        
        else: # Save previous values normally
            # Save and thres prev value
            self.prev_value = self.current_value
            self.prev_value = self.thres(self.prev_value,30)
            
            # Sav and thres prev error
            self.prev_error = self.current_error
            self.prev_error = self.thres(self.prev_error,30)
            

            # Save adn thres total error 
            self.total_error = self.thres(self.total_error,300)
        
        if verbose:
            print("[P: {}, I: {}, D: {}] = {}".format(self.current_error,self.total_error,self.diff,ret))
        
        return round(ret)       

#    def __str__(self):
#        return "[P: {}, I: {},T: {}] = {}".format(self.current_error,self.total_error,self.diff,ret))
    
