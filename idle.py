


################################################################################
#                                    IDLE_ERROR                            
################################################################################

    def idle_error_entry(self):
        self.logI("\n\n< - - - -  ! ATTENTIONS ! - - - - >\n\nThere was an error on the previous state, now is idling. If you want, you can restart from the \"Motor settings state\"\n")

    def idle_error_eval(self):
        # Ricomincia dall'inizio
        if self.restart_fsm.rising():
            self.logI("\n\n < - - - ! IRIS MACHINE IS RESTARTING ! - - - >\n\n")
            self.gotoState("check_connections")

    def idle_error_exit(self):
        pass


################################################################################
#                                    IDLE_STATE                                       ATTENZIONE: In ogni stato attivare s* = 1 al termine !
################################################################################

    def idle_state_entry(self):
        pass

    def idle_state_eval(self):
        # Controllo Enable IRIS_fsm
        if self.enable_fsm.rising():
            self.e = 1
        elif self.enable_fsm.falling():
            self.e = 0
        
        if self.coupling.rising():
            self.coupling_push_botton = 1

        if self.decoupling.rising():
            self.decoupling_push_botton = 1

        # s1 : Charge_Slider_state
        if   self.s1 == 0 and self.s2 == 0 and self.s4 == 0 and self.s5 == 0 and self.s6 == 0 and self.s7 == 0 and self.e == 1 :
            self.gotoState("Charge_Slider_state")
        # s2 : Charge_state    
        elif self.s1 == 1 and self.s2 == 0 and self.s4 == 0 and self.s5 == 0 and self.s6 == 0 and self.s7 == 0 and self.e == 1 :
            self.state_1.put(2)
            if self.state_1.putCompleting():
               self.gotoState("Charge_Central_state") 
        # s4 : Irradiation_state
        elif self.s1 == 1 and self.s2 == 1 and self.s4 == 0 and self.s5 == 0 and self.s6 == 0 and self.s7 == 0 and self.e == 1 and self.coupling_push_botton:
            self.state_2.put(2)
            if self.state_2.putCompleting():
               self.logI("\n\n> - - - - ALIGNMENT / DEPOSITIONING PHASE - - - - <\n\n")    
               self.gotoState("Irradiation_state")
        # s5 : Discharge_Slider_state
        elif self.s1 == 1 and self.s2 == 1 and self.s4 == 1 and self.s5 == 0 and self.s6 == 0 and self.s7 == 0 and self.e == 1 and self.decoupling_push_botton:
            self.state_4.put(2)
            if self.state_4.putCompleting():
               self.coupling.put(0)
               self.gotoState("Discharge_Slider_state")
        # s6 : Discharge_Central_state
        elif self.s1 == 1 and self.s2 == 1 and self.s4 == 1 and self.s5 == 1 and self.s6 == 0 and self.s7 == 0 and self.e == 1 :
            self.state_5.put(2)
            if self.state_5.putCompleting():
               self.gotoState("Discharge_Central_state")
        # s7 : Discharge_Buffer_State
        elif self.s1 == 1 and self.s2 == 1 and self.s4 == 1 and self.s5 == 1 and self.s6 == 1 and self.s7 == 0 and self.e == 1 :
            self.state_6.put(2)
            if self.state_6.putCompleting():
               self.gotoState("Discharge_Buffer_State") # Buffer di scarica 

        # Gestione Fine processo
        elif self.s1 == 1 and self.s2 == 1 and self.s4 == 1 and self.s5 == 1 and self.s6 == 1 and self.s7 == 1 and self.e == 1 :
               self.gotoState("last_state") # fine processo 

        # In questo caso l'intera State Machine si ferma finché non non viene ripremuto l' "Enable" dall'interfaccia, eventualmente si può ricominciare dall'Homing_State         
        elif self.e == 0:
            self.logI("\n < - - - !  MACHINE STOPPED ! - - - >\n   . . .    Iris is waiting   . . . \n")
            
            # Nel caso in cui si desidera un Restart completo della FSM
            if self.restart_fsm.rising():
                self.logI("\n\n < - - - ! IRIS MACHINE IS RESTARTING ! - - - >\n")
                self.n_restart = self.n_restart+1
                # Restart IRIS: Il processo inizia nuovamente dall' Homing_State
                self.gotoState("check_connections")
  
    def idle_state_exit(self):
        pass

 # Gestione fine processo - - - - - - - - - - - - - - - - - - - - - - - - - -
    def last_state_entry(self):
        self.state_7.put(2)
        self.logI("\n\n########################################### IRIS HAS FINISHED ###########################################\n\n")
                                                   

    def last_state_eval(self):           
        if self.restart_fsm.rising():
            self.e = 0
            self.logI("\n\n < - - - ! IRIS MACHINE IS RESTARTING ! - - - >\n")
            self.n_restart = self.n_restart+1
            # Restart IRIS: Il processo inizia nuovamente dall' Homing_State
            self.gotoState("check_connections")

    def  last_state_exit(self):
         pass       
            
