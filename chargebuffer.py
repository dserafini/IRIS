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
    
    # START PROCEDURE
    self.gotoState('chargebuffer_home_forward')
    
    def chargebuffer_home_forward_entry(self): 
      self.m2_home.put(1)
      self.tmrSet('moveTimeout', 10)                                              
      self.logI("charge buffer homing forward...")
    
    def chargebuffer_home_forward_eval(self):
      if self.m2_done_moving.rising():                  
        self.logI("charge buffer in home forward")
        self.gotoState("chargebuffer_home_reverse")
      
      elif self.tmrExpiring("moveTimeout"):                                       
        self.state_2.put(4) 
        self.logI("<  - - - !! ERROR: Charge Buffer movement - - -  >")
        self.gotoState("idle_error")
    
    def chargebuffer_home_forward_exit(self):
      pass


    def chargebuffer_home_reverse_entry(self): 
      self.m2_home.put(1)
      self.tmrSet('moveTimeout', 10)                                              
      self.logI("charge buffer homing reverse...")
    
    def chargebuffer_home_reverse_eval(self):
      if self.m2_done_moving.rising():                  
        self.logI("charge buffer in home reverse")
        self.gotoState("chargebuffer_home_forward") 
      
      elif self.tmrExpiring("moveTimeout"):                                       
        self.state_2.put(4) 
        self.logI("<  - - - !! ERROR: Charge Buffer movement - - -  >")
        self.gotoState("idle_error") 
    
    def chargebuffer_home_reverse_exit(self):
      pass


    def chargebuffer_idle_error_entry(self): 
      pass
    
    def chargebuffer_idle_error_eval(self):
      self.logI("idling due to error")
    
    def chargebuffer_idle_error_exit(self):
      pass
