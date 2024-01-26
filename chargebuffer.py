from smlib import fsmBase
    
class exampleFsm(fsmBase):
  def __init__(self, name, *args, **kwargs):
    super(exampleFsm, self).__init__(name, **kwargs)
    
    
    self.m2_relative = self.connect("WhHrdwMtbx54A_Chan02:Motr.RLV")
    self.m2_home_forward = self.connect("WhHrdwMtbx54A_Chan02:Homf.PROC")
    self.m2_home_reverse = self.connect("WhHrdwMtbx54A_Chan02:Homr.PROC")                
    self.m2_stop = self.connect("WhHrdwMtbx54A_Chan02:Motr.STOP")
    self.m2_done_moving = self.connect("WhHrdwMtbx54A_Chan02:Motr.DMOV")
    self.m2_speed = self.connect("WhHrdwMtbx54A_Chan02:Motr.S")
    self.m2_min_velocity = self.connect("WhHrdwMtbx54A_Chan02:Motr.VBAS")
    
    self.counter = self.connect("testcounter")
    self.mirror = self.connect("testmirror")
    self.enable = self.connect("testenable")
    
    self.gotoState('idle')
    
    # idle state
    def idle_eval(self):
      if self.enable.rising():
        self.gotoState("mirroring")
    
    # mirroring state
    def mirroring_eval(self):
      if self.enable.falling():
        self.gotoState("idle")
      elif self.counter.changing():
        readValue = self.counter.val()
        self.mirror.put(readValue)
