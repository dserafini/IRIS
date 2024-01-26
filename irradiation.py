

################################################################################
#                             Irradiation_state                                     
################################################################################    

# s4: Allineamento Targets con la linea di fascio, Motore: m5, m6

# Accoppiamnto con il fascio- - - - - - - - - - - - - - - - - - -

# inizializzazione interfaccia 

    def Irradiation_state_entry(self):
        self.state_4.put(1)
        self.decoupling.put(0)                                                 
        self.tmrSet('moveTimeout19_1', 10)                                            # Set a timer of 10s

    def Irradiation_state_eval(self):   
        if self.decoupling.putCompleting():       
               self.gotoState("Irradiation_coupling")                                   

        elif self.tmrExpiring("moveTimeout19_1"):                                     # Timer expired event
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Coupling_Motor movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Irradiation_state_exit(self):
         pass  

# Coupling 
    def Irradiation_coupling_entry(self):
        self.m6_coupling.put(1)                                                       # COUPLING MOTOR                                                    
        self.tmrSet('moveTimeout19', 10)                                              # Set a timer of 10s
        self.logI("\tStart coupling to beamline")

    def Irradiation_coupling_eval(self):
        if self.m6_done_moving.rising():            
            self.logI("\t> - Coupled to beamline - <")
            self.gotoState("wait1")                                   

        elif self.tmrExpiring("moveTimeout19"):                                       # Timer expired event
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Coupling_Motor movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Irradiation_coupling_exit(self):
         pass  

# Stato di attesa (1)- - - - - - - - - - - - - - - - - - -
    def wait1_entry(self):
        self.logI("\t. . . Ready for allineating 1^ Target . . .")

    def wait1_eval(self):
        if self.align_disk1.rising():
            self.gotoState("Irradiation_state1")

        if self.finish.rising(): 
            self.gotoState("irradiation_finished") 
        
    def  wait1_exit(self):
         pass   

# Stato fine "Irradiation state"
    def irradiation_finished_entry(self):
        self.state_4.put(2)   

    def irradiation_finished_eval(self):
        if self.state_4.putCompleting():                                                
            self.gotoState("idle_state")   
        
    def  irradiation_finished_exit(self):
        self.s4 = 1                                                                   # Quarto stato completato



# Allineamento 1^ Target - - - - - - - - - - - - - - -           
    def Irradiation_state1_entry(self):
        self.m5_relative.put(64*self.align_d1)
        self.tmrSet('moveTimeout20', 30)                                              # Set a timer of 30s
        self.logI("\tStarting Central movement")

    def Irradiation_state1_eval(self):
        if self.m5_done_moving.rising():                                              # la variabile "self.d1" permette di eseguire una volta sola l' "if"
            self.logI("\t> 1^ - Target Allineated <")
            self.d1 = 1                                                               # 1^ Target Allineato
            self.gotoState("wait2") 

        elif self.tmrExpiring("moveTimeout20"):                                       # Timer expired event
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement: Alineating 1^ Target - - -  >\n")
            self.gotoState("idle_error")  
 
    def  Irradiation_state1_exit(self):
         pass    
 
# Stato di attesa (2) - - - - - - - - - - - - - - - - - - -
    def wait2_entry(self):
        self.logI("\t. . . Ready for allineating 2^ Target . . .")

    def wait2_eval(self):
        if self.align_disk1.rising():
            self.gotoState("Irradiation_state2")

        if self.finish.rising(): 
            self.gotoState("irradiation_finished") 
        
    def  wait2_exit(self):
        pass  

# Allineamento 2^ Target - - - - - - - - - - - - - - -           
    def Irradiation_state2_entry(self):
        self.m5_relative.put(64*self.align_d2)
        self.tmrSet('moveTimeout21', 30)                                             # Set a timer of 30s
        self.logI("\tStarting Central movement")
        
    def Irradiation_state2_eval(self):
        if self.m5_done_moving.rising():                                             # la variabile "self.d2" permette di eseguire una volta sola l' "if"
            self.logI("\t > 2^ - Target Allineated <")
            self.d2 = 1                                                              # 2^ Target Allineato
            self.gotoState("wait3")

        elif self.tmrExpiring("moveTimeout21"):                                      # Timer expired event
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement: Alineating 2^ Target - - -  >\n")
            self.gotoState("idle_error")  

    def  Irradiation_state2_exit(self):
         pass
    
# Stato di attesa (3) - - - - - - - - - - - - - - - - - - -
    def wait3_entry(self):
        self.logI("\t. . . Ready for allineating 3^ Target . . .")

    def wait3_eval(self):
        if self.align_disk1.rising():
            self.gotoState("Irradiation_state3")

        if self.finish.rising(): 
            self.gotoState("irradiation_finished") 
 
    def  wait3_exit(self):
        pass 

# Allineamento 3^ Target - - - - - - - - - - - - - - -           
    def Irradiation_state3_entry(self):
        self.m5_relative.put(64*self.align_d3)
        self.tmrSet('moveTimeout22', 30)                                             # Set a timer of 30s
        self.logI("\tStarting Central movement")

    def Irradiation_state3_eval(self):
        if self.m5_done_moving.rising():                                             # la variabile "self.d3" permette di eseguire una volta sola l' "if"
            self.logI("\t > 3^ - Target Allineated <")
            self.d3 = 1                                                              # 3^ Target Allineato
            self.gotoState("wait4")

        elif self.tmrExpiring("moveTimeout22"):                                      # Timer expired event
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement: Alineating 3^ Target - - -  >")
            self.gotoState("idle_error")  
  
    def  Irradiation_state3_exit(self):
         pass 

# Stato di attesa (4): Fine - - - - - - - - - - - - - - - - - - -
    def wait4_entry(self):
        self.logI("\t. . . Iris is waiting: Is Deposition phase finished ? . . .\n")

    def wait4_eval(self):

        if self.finish.rising(): 
            self.gotoState("finish_homing") 

    def  wait4_exit(self):
        pass 

# Homing Central - - - - - - - - - - - - - - - - - - - - -
    def finish_homing_entry(self):
        self.m5_home.put(1)
        self.tmrSet('moveTimeout23_1', 30)             
        self.logI("\t< Homing Cental Movement >")

    def finish_homing_eval(self):
        if self.m5_done_moving.rising():                                                          
            self.gotoState("finish_interface")                 

        elif self.tmrExpiring("moveTimeout23_1"):
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement - - -  >\n")
            self.gotoState("idle_error")

    def finish_homing_state4_exit(self):
        pass

# Segnalazione interfaccia fine processo di irradiazione 
    def finish_interface_entry(self):
        self.state_4.put(2) 

    def finish_interface_eval(self):
        if self.state_4.putCompleting():                                                     
            self.gotoState("idle_state")                 

        elif self.tmrExpiring("moveTimeout23"):
            self.state_4.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement - - -  >\n")
            self.gotoState("idle_error")

    def finish_interface_exit(self):
        self.s4 = 1                                                                 # Quarto stato completato 
