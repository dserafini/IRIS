

################################################################################
#                           Discharge_Slider_state                                       
################################################################################    

# s5: Posizionamento dello slider e per l'accoppiamento "Discharge_Slider", Motore: m3, m6

#Segnalazione interfaccia
    def Discharge_Slider_state_entry(self):
        self.state_5.put(1)                                              
        self.tmrSet('moveTimeout24', 10)                                             # Set a timer of 30s

    def Discharge_Slider_state_eval(self):
        if self.state_5.putCompleting():            
            self.gotoState("Decoupling_state")                                       # Torna nello stato principale: "idle_state"

        elif self.tmrExpiring("moveTimeout24"):                                      # Timer expired event
            self.state_5.put(4)
            self.logI("\t<  - - - !! ERROR: Decoupling_Motor movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Discharge_Slider_state_exit(self):
         pass  

#Decoupling
    def Decoupling_state_entry(self):
        self.m6_decoupling.put(1)                                                    # DECOUPLING MOTOR                                                    
        self.tmrSet('moveTimeout24', 20)                                             # Set a timer of 30s
        self.logI("\tStart decoupling to beamline")

    def Decoupling_state_eval(self):
        if self.m6_done_moving.rising():            
            self.logI("\t> - Decoupled to beamline - <\n")
            self.gotoState("alligne_2_m6")                                           # Torna nello stato principale: "idle_state"

        elif self.tmrExpiring("moveTimeout24"):                                      # Timer expired event
            self.state_5.put(4)
            self.logI("\t<  - - - !! ERROR: Decoupling_Motor movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Decoupling_state_exit(self):
         pass  

    # 2^ Allineamento longitudinal axis: m6 -
        
    def alligne_2_m6_entry(self):
        self.m6_absolute.put(self.exact_longitudinal_axis_steps)
        self.tmrSet('moveTimeout3_1', 30)             

    def alligne_2_m6_eval(self):
        if self.m6_absolute.putCompleting():            
            self.logI("\tLongitudinal motor allineated...")
            self.gotoState("Discharge_Slider_state2")                 

        elif self.tmrExpiring("moveTimeout3_1"):
            self.logI("\t<  - - - !! ERROR: Decoupling_Motor movement - - -  >\n")
            self.state_0.put(4)
            if self.state_0.putCompleting():
               self.gotoState("idle_error")

    def alligne_2_m6_exit(self):
        pass

# Discharge slider - - - - - - - - - - - - - - - - -
    def Discharge_Slider_state2_entry(self):
        self.m3_absolute.put(self.exact_discharge_slider_steps)                       # Posizionamento Charge Slider                                                      
        self.tmrSet('moveTimeout25', 50)                                              # Set a timer of 30s
        self.logI("\nStarting Discharge_Slider movement")

    def Discharge_Slider_state2_eval(self):
        if self.m3_absolute.putCompleting():            
            self.logI("\t> - Discharge Slider out - <\n")
            self.gotoState("state5_end")                                              # Torna nello stato principale: "idle_state"

        elif self.tmrExpiring("moveTimeout25"):                                       # Timer expired event
            self.state_5.put(4)
            self.logI("\t<  - - - !! ERROR: Discharge_Slider movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Discharge_Slider_state2_exit(self):
         pass      
    
# Fine stato 5 
    def state5_end_entry(self):
        self.state_5.put(2)                                        
        self.tmrSet('moveTimeout25', 10)                                              

    def state5_end_eval(self):
        if self.state_5.putCompleting():                                                            
            self.gotoState("idle_state")                                              # Torna nello stato principale: "idle_state"

        elif self.tmrExpiring("moveTimeout25"):                                       # Timer expired event
            self.state_5.put(4)
            self.logI("\t<  - - - !! ERROR: Discharge_Slider movement - - -  >\n")
            self.gotoState("idle_error")  

    def  state5_end_exit(self):
        self.s5 = 1                                                                    # Quinto stato completato

################################################################################
#                           Discharge_Central_state                                      
################################################################################   

# s6 : Carica "Central Movement", Motore: m5 , Attuatore: "lock1" 

# Gestione interfaccia 
    def Discharge_Central_state_entry(self):
        self.state_6.put(1)      
        self.tmrSet('moveTimeout26_1', 10)                                              

    def Discharge_Central_state_eval(self):
        if self.state_6.putCompleting():      
            self.gotoState("Extract_lock1_1") 

        elif self.tmrExpiring("moveTimeout26_1"):                                 
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  Discharge_Central_state_exit(self):
         pass      

# Estrazzione attuatore "lock1"
    def Extract_lock1_1_entry(self):
        self.lock1_extract.put(1)                                                        # Estrae Solenoid_Bottom (1)      
        self.tmrSet('moveTimeout26_2', 10)                                             

    def Extract_lock1_1_eval(self):
        if self.lock1_extract.putCompleting():      
            self.gotoState("Discharge_Central_state1") 

        elif self.tmrExpiring("moveTimeout26_2"):                                 
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  Extract_lock1_1_exit(self):
         pass   

# Scarica 1^ Target - - - - - - - - - - - - - - - - - - - - - 

# Movimentazione motore
    def Discharge_Central_state1_entry(self):
        self.logI("\t> - - - Discharging Central Movement - - - < ")                               
        self.m5_absolute.put(self.exact_1_discharge_central_steps + self.relative_correction)                                           # 1^ Movimentazione: Scarica primo Target
        self.tmrSet('moveTimeout26_3', 30)                                             
        self.logI("\tStarting Central movement")   

    def Discharge_Central_state1_eval(self):
        if self.m5_done_moving.rising():      
            self.gotoState("Discharge_Central_correction1") 

        elif self.tmrExpiring("moveTimeout26_3"):                                       
            self.state_6.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Central_state1_exit(self):
         pass      

# Correzzione passi
    def Discharge_Central_correction1_entry(self):
        self.m5_absolute.put(self.exact_1_discharge_central_steps - self.relative_correction)                                             # 1^ Correzione                     
        self.tmrSet('moveTimeout26', 40)                                              
        self.logI("\tStarting Central movement")   

    def Discharge_Central_correction1_eval(self):
        if self.m5_done_moving.rising():      
            self.gotoState("solenoid_bottom_1") 

        elif self.tmrExpiring("moveTimeout26"):                                      
            self.state_6.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ correction movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Central_correction1_exit(self):
         pass

# Insert solenoid bottom, lock1
    def solenoid_bottom_1_entry(self):   
        self.lock1_insert.put(1)                                                                                                           # "Disinserisce" Solenoid_Bottom (1)                                    
        self.tmrSet('moveTimeout27', 10)                                              

    def solenoid_bottom_1_eval(self):
        if self.lock1_insert.putCompleting():          
            self.gotoState("Extract_lock1_2") 
  
        elif self.tmrExpiring("moveTimeout27"):                                      
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  solenoid_bottom_1_exit(self):
         time.sleep(2)  


# Scarica 2^ Target - - - - - - - - - - - - - - - - - - - - -

# Estrazione attuatore "lock1"
    def Extract_lock1_2_entry(self):
        self.lock1_extract.put(1)                                                                                                           # Estrae Solenoid_Bottom (2)      
        self.tmrSet('moveTimeout28_2', 10)                                             

    def Extract_lock1_2_eval(self):
        if self.lock1_extract.putCompleting():      
            self.gotoState("Discharge_Central_state2") 

        elif self.tmrExpiring("moveTimeout28_2"):                                 
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  Extract_lock1_2_exit(self):
         pass   

# Movementazione motore - scarica 2^ Target
    def Discharge_Central_state2_entry(self):       
        self.m5_absolute.put(self.exact_2_discharge_central_steps + self.relative_correction)                                                # 2^ Movimentazione: Scarica secondo Target
        self.tmrSet('moveTimeout28', 30)                                              
        self.logI("\tStarting Central movement")   

    def Discharge_Central_state2_eval(self):
        if self.m5_done_moving.rising():          
            self.logI("\t  2^ - Target out \n")                             
            self.gotoState("Discharge_Central_correction2") 
 
        elif self.tmrExpiring("moveTimeout28"):                                       
            self.state_6.put(4)
            self.logI("\n<  - - - !! ERROR: Central movement : during 2^ movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Central_state2_exit(self):
         pass

# Correzzione passi
    def Discharge_Central_correction2_entry(self):
        self.m5_absolute.put(self.exact_2_discharge_central_steps - self.relative_correction)                                                  # Correzione posizione per insert solenaoid bottom                      
        self.tmrSet('moveTimeout28_1', 30)                                              
        self.logI("\tStarting Central movement")   

    def Discharge_Central_correction2_eval(self):
        if self.m5_done_moving.rising():      
            self.gotoState("solenoid_bottom_2") 

        elif self.tmrExpiring("moveTimeout28_1"):                                      
            self.state_6.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ correction movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Central_correction2_exit(self):
         pass      

# Insert solenoid bottom, lock1
    def solenoid_bottom_2_entry(self):   
        self.lock1_insert.put(1)                                                                                                               # "Disinserisce" Solenoid_Bottom (2)                                    
        self.tmrSet('moveTimeout28_4', 10)                                              

    def solenoid_bottom_2_eval(self):
        if self.lock1_insert.putCompleting():          
            self.gotoState("Extract_lock1_3") 
  
        elif self.tmrExpiring("moveTimeout28_4"):                                      
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  solenoid_bottom_2_exit(self):
         time.sleep(2)  


# Scarica 3^ Target - - - - - - - - - - - - - - - - - - - - -

# Estrazione attuatore "lock1"
    def Extract_lock1_3_entry(self):
        self.lock1_extract.put(1)                                                                                                             # Estrae Solenoid_Bottom (3)      
        self.tmrSet('moveTimeout29_1', 10)                                             

    def Extract_lock1_3_eval(self):
        if self.lock1_extract.putCompleting():      
            self.gotoState("Discharge_Central_state3") 

        elif self.tmrExpiring("moveTimeout29_1"):                                 
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  Extract_lock1_3_exit(self):
         time.sleep(1)   

# Movimentazione motore 
    def Discharge_Central_state3_entry(self):      
        self.m5_absolute.put(self.exact_3_discharge_central_steps + self.relative_correction)                                                  # 3^ Movimentazione: Scarica terzo Target
        self.tmrSet('moveTimeout30', 30)                                              
        self.logI("\tStarting Central movement")   

    def Discharge_Central_state3_eval(self):
        if self.m5_done_moving.rising():          
            self.logI("\t3^ - Target out")
            self.gotoState("Discharge_Central_correction3")                              

        elif self.tmrExpiring("moveTimeout30"):                                     
            self.state_6.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 3^ movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Central_state3_exit(self):
         pass

# Correzzione passi
    def Discharge_Central_correction3_entry(self):
        self.m5_absolute.put(self.exact_3_discharge_central_steps - self.relative_correction)                                                   # Correzione posizione per insert solenaoid bottom                      
        self.tmrSet('moveTimeout30_1', 30)                                              
        self.logI("\tStarting Central movement")   

    def Discharge_Central_correction3_eval(self):
        if self.m5_done_moving.rising():      
            self.gotoState("solenoid_bottom_3") 

        elif self.tmrExpiring("moveTimeout30_1"):                                      
            self.state_6.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ correction movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Central_correction3_exit(self):
         pass      

# Insert lock1
    def solenoid_bottom_3_entry(self):       
        self.lock1_insert.put(1)                                                                                                                 # "Disinserisce" Solenoid_Bottom (2)
        self.tmrSet('moveTimeout31', 30)                                              

    def solenoid_bottom_3_eval(self):
        if self.lock1_insert.putCompleting():                  
            self.gotoState("final_discharge_state")    
 
        elif self.tmrExpiring("moveTimeout31"):                                         
            self.state_6.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 2^ movement - - -  >")
            self.gotoState("idle_error")  

    def  solenoid_bottom_3_exit(self):
         pass

# Stato finale - segnalazione interfaccia
    def final_discharge_state_entry(self):       
        self.state_6.put(2)                                                             
        self.tmrSet('moveTimeout31_1', 10)                                              

    def final_discharge_state_eval(self):
        if self.state_6.putCompleting():                                                                 
            self.gotoState("idle_state")    
 
        elif self.tmrExpiring("moveTimeout31_1"):                                       
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  final_discharge_state_exit(self):
         self.s6 = 1                                                                                                                               # Sesto stato completato

################################################################################
#                           Discharge_Buffer_State                                      
################################################################################   

# s7: Carica e Scarica del "Discharge_Buffer", Motore: m4

# Segnalazione interfaccia stato 7
    def Discharge_Buffer_State_entry(self):
        self.state_7.put(1)
        self.tmrSet('moveTimeout32', 10)                                     

    def Discharge_Buffer_State_eval(self):
        if self.state_7.putCompleting():            
            self.gotoState("Discharge_Buffer_State2")

        elif self.tmrExpiring("moveTimeout32"):                                     
            self.state_7.put(4)
            self.gotoState("idle_error")  

    def  Discharge_Buffer_State_exit(self):
         pass 

# Carica "Discharge_Buffer" - - - - - - - - - - - - - - - - - - - - -
    def Discharge_Buffer_State_entry(self):
        self.m4_reverse.put(1)                                                       # Carica dei 3 Target con un giro completo
        self.tmrSet('moveTimeout32', 30)                                             
        self.logI("\tStarting Discharge Buffer movement")

    def Discharge_Buffer_State_eval(self):
        if self.m4_done_moving.rising():            
            self.logI("\t> \"  Discharge_Buffer \" Charged <")
            self.gotoState("Discharge_Buffer_State2")

        elif self.tmrExpiring("moveTimeout32"):                                   
            self.state_7.put(4)
            self.logI("\t<  - - - !! ERROR: Discharge Buffer movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Buffer_State_exit(self):
         pass 
    
# Scarica1 "Discharge_Buffer" - - - - - - - - - - - - - - - - - - - - -
    def Discharge_Buffer_State2_entry(self):
        self.m4_home.put(1)                                                          # Scarica dei 3 Target con un giro completo (mediante homing)
        self.tmrSet('moveTimeout33', 30)                                            
        self.logI("\tStarting Discharge Buffer movement")

    def Discharge_Buffer_State2_eval(self):
        if self.m4_done_moving.rising():            
            self.gotoState("Discharge_Buffer_State3")

        elif self.tmrExpiring("moveTimeout33"):                                      # Timer expired event
            self.state_7.put(4)
            self.logI("\t<  - - - !! ERROR: Charge Buffer movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Discharge_Buffer_State2_exit(self):
         pass
    
# Scarica2 "Discharge_Buffer" - - - - - - - - - - - - - - - - - - - - -
    def Discharge_Buffer_State3_entry(self):
        self.m4_reverse.put(1)                                                       # Scarica dei 3 Target con un giro completo (mediante homing)
        self.tmrSet('moveTimeout34', 30)                                             # Set a timer of 50s
        self.logI("\tStarting Discharge Buffer movement")

    def Discharge_Buffer_State3_eval(self):

        if self.m4_done_moving.rising():            
            self.logI("\t> - !\"  Discharge_Buffer \"  Discharged ! - <")
            self.s7 = 1                                                              # Settimo stato completato
            self.state_7.put(2)
            self.gotoState("idle_state")

        elif self.tmrExpiring("moveTimeout34"):                                      # Timer expired event
            self.state_7.put(4)
            self.logI("\t<  - - - !! ERROR: Charge Buffer movement - - -  >")
            self.gotoState("idle_error")  

    def  Discharge_Buffer_State3_exit(self):
        if self.n_restart%2 == 0:  # se pari
             self.bufferpos.put(1)                                                    # Per la carica di 1 compressa
        else:                      # se dispari
           self.bufferpos.put(0)
        
# committed 1-09
