from smlib import fsmBase

class centralmovement_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(centralmovement_fsm, self).__init__(name, **kwargs)

        self.m5_relative = self.connect("WhHrdwMtbx54A_Chan05:Motr.RLV")
        self.m5_home_forward = self.connect("WhHrdwMtbx54A_Chan05:Homf.PROC")
        self.m5_home_reverse = self.connect("WhHrdwMtbx54A_Chan05:Homr.PROC")                
        self.m5_stop = self.connect("WhHrdwMtbx54A_Chan05:Motr.STOP")
        self.m5_done_moving = self.connect("WhHrdwMtbx54A_Chan05:Motr.DMOV")
        self.m5_speed = self.connect("WhHrdwMtbx54A_Chan05:Motr.S")
        self.m5_min_velocity = self.connect("WhHrdwMtbx54A_Chan05:Motr.VBAS")

        self.home = self.connect("FeExprIris01A_Proc02:Enab")

        self.gotoState('idle')

    # idle state
    def idle_eval(self):
        if self.home.rising():
            self.gotoState("home_forward")
        if self.home.falling():
            self.gotoState("home_reverse")

    # home forward state
    def home_forward_entry(self):
        self.m5_home_forward.put(1)
        
    def home_forward_eval(self):
        if self.m5_done_moving.rising():
            self.logI('central movement in home forward')
            self.gotoState('idle')
            
    def home_forward_exit(self):
        pass

    # home reverse state
    def home_reverse_entry(self):
        self.m5_home_reverse.put(1)
        
    def home_reverse_eval(self):
        if self.m5_done_moving.rising():
            self.logI('central movement in home reverse')
            self.gotoState('idle')
            
    def home_reverse_exit(self):
        pass
