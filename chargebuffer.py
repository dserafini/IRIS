from smlib import fsmBase

class chargebuffer_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(chargebuffer_fsm, self).__init__(name, **kwargs)

        self.m2_relative = self.connect("WhHrdwMtbx54A_Chan02:Motr.RLV")
        self.m2_home_forward = self.connect("WhHrdwMtbx54A_Chan02:Homf.PROC")
        self.m2_home_reverse = self.connect("WhHrdwMtbx54A_Chan02:Homr.PROC")                
        self.m2_stop = self.connect("WhHrdwMtbx54A_Chan02:Motr.STOP")
        self.m2_done_moving = self.connect("WhHrdwMtbx54A_Chan02:Motr.DMOV")
        self.m2_speed = self.connect("WhHrdwMtbx54A_Chan02:Motr.S")
        self.m2_min_velocity = self.connect("WhHrdwMtbx54A_Chan02:Motr.VBAS")

        self.home = self.connect("FeExprIris01A_Proc01:home")

        self.gotoState('idle')

    # idle state
    def idle_eval(self):
        if self.home.rising():
            self.gotoState("home_forward")
        if self.home.falling():
            self.gotoState("home_reverse")

    # home forward state
    def home_forward_entry(self):
        self.m2_home_forward.put(1)
        
    def home_forward_eval(self):
        if self.m2_done_moving.rising():
            self.logI('charge slider in home forward')
            self.gotoState('idle')
            
    def home_forward_exit(self):
        pass

    # home reverse state
    def home_reverse_entry(self):
        self.m2_home_reverse.put(1)
        
    def home_reverse_eval(self):
        if self.m2_done_moving.rising():
            self.logI('charge slider in home reverse')
            self.gotoState('idle')
            
    def home_reverse_exit(self):
        pass
