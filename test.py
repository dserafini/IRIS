#! /usr/bin/python
from smlib import fsmBase, loader

# FSM definition
class exampleFsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(exampleFsm, self).__init__(name, **kwargs)

        self.counter = self.connect("testcounter")
        self.mirror = self.connect("testmirror")
        self.enable = self.connect("testenable")

        self.var16 = self.connect("FeExprIris01A_Proc17:Enab")
        self.var17 = self.connect("FeExprIris01A_Proc17:Enab")

        self.gotoState('idle')

    # idle state
    def idle_entry(self):
        self.logI("idling")
        
    def idle_eval(self):
        if self.var16.rising():
            self.var16.put(0)
            self.gotoState("mirroring")

    # mirroring state
    def mirroring_entry(self):
        self.var17.put(2)
        
    def mirroring_eval(self):
        if self.var17.putCompleting():
            self.logI("var17 is " + self.var17.val(as_string=True))
            self.gotoState("idle")

# Main
if __name__ == '__main__':
    # load the fsm
    l = loader()
    l.load(exampleFsm, "myFirstFsm")

    # start execution
    l.start()
