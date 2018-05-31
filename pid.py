# -*- coding: utf-8 -*-

import time

class PID:
    """ Simple PID control.

        This class implements a simplistic PID control algorithm. When
        first instantiated all the gain variables are set to zero, so
        calling the method GenOut will just return zero.
    """
    def __init__(self,step_size='eighth'):
        # initialize gains
        self.Kp = 0
        self.Kd = 0
        self.Ki = 0
        self.sp = 0.0
        
        if step_size == 'eighth':
            self.arcs_deg = 23.1428
        if step_size == 'half':
            self.arcs_deg = 92.5714
        
        self.pos2deg = self.arcs_deg/3600  #mulitply PTU steps by this to convert from PTU steps to degrees
        self.deg2pos = 3600/23.1428  #multiply degrees by this to convert from degrees to PTU steps

        self.Initialize()

    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr
        
    def SetSP(self, sp):
        """ Change setpoint (offset) """
        self.sp = sp

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0


    def GenOut(self, ss_off_x):
        """ Performs a PID computation and returns a control value based
            on the elapsed time (dt) and the error signal from a summing
            junction (the error parameter).
        """
        error = self.sp + ss_off_x 
        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        de = error - self.prev_err              # get delta error

        self.Cp = self.Kp * error               # proportional term
        self.Ci += error * dt                   # integral term

        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     # derivative term

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error
        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)