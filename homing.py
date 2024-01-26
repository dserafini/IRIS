
################################################################################
#                               HOMING MOTORS
################################################################################

# Homing iniziale: solo per la prima esecuzione

# Charge Slider: m1 -
    def homing_entry(self):
        self.state_0.put(1)
        self.tmrSet('moveTimeout1_1', 3)  

    def homing_eval(self):
        if self.state_0.putCompleting():           
            self.gotoState("homing1")                 

        elif self.tmrExpiring("moveTimeout1_1"):
            self.logI("\tERROR: putCompleting Homing state\t")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")
            
    def homing_exit(self):
        pass

    def homing1_entry(self):
         self.m1_home.put(1)
         self.logI("\t< - - - -   HOMING_STATE STARTS   - - - - >\n\t< Homing Charge Slider >\t")

         self.tmrSet('moveTimeout1', 30)  

    def homing1_eval(self):
        if self.m1_done_moving.rising():            
            self.logI("\tHoming Charge Slider,    Done!\t")
            self.gotoState("homing_DS")                 

        elif self.tmrExpiring("moveTimeout1"):
            self.logI("\tERROR: Homing Charge_Slider")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")
            
    def homing1_exit(self):
        pass
    # - -

    # Discharge Slider: m3 -
        
    def homing_DS_entry(self):
        self.m3_home.put(1)
        self.tmrSet('moveTimeout4', 30)             
        self.logI("\t< Homing Discharge Slider >\t")

    def homing_DS_eval(self):
        if self.m3_done_moving.rising():            
            self.logI("\tHoming Discharge Slider, Done!\t")
            self.gotoState("homing_CDM")                 

        elif self.tmrExpiring("moveTimeout4"):
            self.logI("ERROR: Homing Discharge_Slider")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_DS_exit(self):
        pass
    # - -

    # Couple/Decouple motor: m6 -
    def homing_CDM_entry(self):
        self.coupling.put(0)
        self.tmrSet('moveTimeout5_1', 30)

    def homing_CDM_eval(self):
        if self.coupling.putCompleting():          
            self.gotoState("homing_CDM1")                 

        elif self.tmrExpiring("moveTimeout5_1"):
            self.logI("\tERROR: putCompleting Homing_CDM state")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_CDM_exit(self):
        pass

    def homing_CDM1_entry(self):
        self.m6_decoupling.put(1)
        self.logI("\t< Homing Longitudianl Motor >\t") #modifica
        self.tmrSet('moveTimeout5', 30)

    def homing_CDM1_eval(self):
        if self.m6_done_moving.rising():            
            self.logI("\tDecoupled, Done!\t")
            self.gotoState("homing_DB")                 

        elif self.tmrExpiring("moveTimeout5"):
            self.logI("\tERROR: Homing Decoupling Motor")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_CDM1_exit(self):
        pass
    # - -

    # Discharge Buffer: m4 -
        
    def homing_DB_entry(self):
        self.m4_home.put(1)
        self.tmrSet('moveTimeout6', 50)             
        self.logI("\t< Homing Discharge Buffer >\t")

    def homing_DB_eval(self):
        if self.m4_done_moving.rising():            
            self.logI("\tHoming Discharge Buffer, Done!\t")
            self.gotoState("homing_correction_CM")                 

        elif self.tmrExpiring("moveTimeout6"):
            self.logI("\tERROR: Homing Discharge_Buffer")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")


    def homing_DB_exit(self):
        pass
    # - -


    # Gestione possibile problema Central Movement: m5 attuatori
    def homing_correction_CM_entry(self):
        self.lock1_insert.put(1)                          
        self.tmrSet('moveTimeout3_0', 10)             

    def homing_correction_CM_eval(self):
        if self.lock1_insert.putCompleting():            
            self.gotoState("homing_correction_CM1")                 

        elif self.tmrExpiring("moveTimeout3_0"):
            self.logI("\tERROR: Insert Lock1 ")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_correction_CM_exit(self):
        pass    

    def homing_correction_CM1_entry(self):                          
        self.lock2_insert.put(1)
        self.tmrSet('moveTimeout3_1', 10)             

    def homing_correction_CM1_eval(self):
        if self.lock2_insert.putCompleting():            
            self.gotoState("homing_correction_CM2")                 

        elif self.tmrExpiring("moveTimeout3_1"):
            self.logI("\tERROR: Insert Lock2 ")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_correction_CM1_exit(self):
        pass

    def homing_correction_CM2_entry(self):
           self.m5_relative.put(64*30) 
           self.logI("\t< Homing Cental Movement >\t")
           self.tmrSet('moveTimeout3_2', 30)             

    def homing_correction_CM2_eval(self):
        if self.m5_done_moving.rising():            
            self.logI("\tHoming Central Movement, Done!\t")
            self.gotoState("homing_CM")                 

        elif self.tmrExpiring("moveTimeout3_2"):
            self.logI("\tERROR: Homing Central_Movement")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_correction_CM2_exit(self):
        pass
   
    # Cental Movement: m5 e Attuatori -
        
    def homing_CM_entry(self):
        self.m5_home.put(1)
        self.tmrSet('moveTimeout3', 30)             
        self.logI("\t< Homing Cental Movement >\t")

    def homing_CM_eval(self):
        if self.m5_done_moving.rising():            
            self.logI("\tHoming Central Movement, Done!")
            self.logI("\t< - - - -   HOMING_STATE ENDS   - - - - >\t")
            self.state_0.put(2)
            self.gotoState("alligne_m6")                 

        elif self.tmrExpiring("moveTimeout3"):
            self.logI("\tERROR: Homing Central_Movement")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_CM_exit(self):
        pass


    # Allineamento longitudinal axis: m6 -
        
    def alligne_m6_entry(self):
        self.m6_absolute.put(self.exact_longitudinal_axis_steps)
        self.tmrSet('moveTimeout3_0', 30)             
        self.logI("\t< Homing Cental Movement >\t")

    def alligne_m6_eval(self):
        if self.m6_absolute.putCompleting():            
            self.logI("\tLongitudinal motor allineated...")
            self.logI("\t< - - - -   HOMING_STATE ENDS   - - - - >\t")
            self.state_0.put(2)
            self.gotoState("homing_finished")                 

        elif self.tmrExpiring("moveTimeout3_0"):
            self.logI("\tERROR: Homing Central_Movement")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def alligne_m6_exit(self):
        pass

  # Fine Homing - Segnalazione nell'interfaccia 
    def homing_finished_entry(self):
        self.state_0.put(2)
        self.tmrSet('moveTimeout3_3', 10) 

    def homing_finished_eval(self):
        if self.state_0.putCompleting():
            self.gotoState("idle_state")                 

        elif self.tmrExpiring("moveTimeout3_3"):
            self.logI("\tERROR: homing_finished")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def homing_finished_exit(self):
        time.sleep(1)
    
    # - -    
