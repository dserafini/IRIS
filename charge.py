
import time
from smlib import fsmBase

class charge_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(charge_fsm, self).__init__(name, **kwargs)

        
        # ALIGNMENT - - - - - - - - - - - - - - - - - - - - - - - -
        # Charge_Slider positions
        self.exact_charge_slider_steps = 6656     # 6592+64
        self.exact_discharge_slider_steps = -6720 # -6336+64
        self.exact_longitudinal_axis_steps = 128

        # Charge_Central_Movement position
        self.exact_1_charge_central_steps = 8064  # 7872
        self.exact_2_charge_central_steps = 16704 # 16640
        self.exact_3_charge_central_steps = 24960 

        # Discharge Central Movement position
        self.exact_1_discharge_central_steps = 4096
        self.exact_2_discharge_central_steps = 13248-64
        self.exact_3_discharge_central_steps = 21568 # 21568

        # Charge Central Movement Correction
        self.relative_correction = 64  # correzione di uno step

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        # ALIGNMENT " Irradiation_state ": Allineamento Targets con il fascio - - - -
        self.i3 = 64
        
        self.align_d1 = 64
        self.align_d2 = 134                                                                   # VALORI DA VERIFICARE !!
        self.align_d3 = 132 
        # - - - - - - - - - - - - - - - - - - - - - - - -


        # Dichiarazione variabili interne - - - - - - - -

        self.e = 0         # Enable / Disableself.i1
        self.n_restart = 0 # Contegio dei restart per la carica del Charge Buffer

        # Coupling/Decoupling to Beemline

        self.coupling_push_botton = 0
        self.decoupling_push_botton = 0

        # Controlo stati 
        
        self.s1 = 0  # Charge_Slider_state 
        self.s2 = 0  # Charge_Buffer_state
        self.s3 = 0  # Charge_Central_state
        self.s4 = 0  # Irraggiamento_state
        self.s5 = 0  # Discharge_Slider_state
        self.s6 = 0  # Discharge_Central_state
        self.s7 = 0  # Discharge_Buffer_State
        
        # Sliders
        self.l1 = 0
        self.l2 = 0

        # Irradiation_state: Alignment Targets
        self.d1 = 0  # 1^ Target
        self.d2 = 0  # 2^ Target
        self.d3 = 0  # 3^ Target
        
        # - - - - - - - - - - - - - - - - - - - - - - - - - 
        
################################################################################
#          INIZIALIZZAZZIONE MOTORI / PV - LOCKS / PV PROCEDURES
################################################################################

        # Ch1 - Charge Slider motor
        self.m1_relative = self.connect("WhHrdwMtbx54A_Chan01:Motr.RLV")
        self.m1_forward = self.connect("WhHrdwMtbx54A_Chan01:Homf.PROC")
        self.m1_absolute = self.connect("WhHrdwMtbx54A_Chan01:Motr")
        self.m1_home = self.connect("WhHrdwMtbx54A_Chan01:Homr.PROC")                # Per Homing
        self.m1_stop = self.connect("WhHrdwMtbx54A_Chan01:Motr.STOP")
        self.m1_done_moving = self.connect("WhHrdwMtbx54A_Chan01:Motr.DMOV")
        self.m1_speed = self.connect("WhHrdwMtbx54A_Chan01:Motr.S")
        self.m1_min_velocity = self.connect("WhHrdwMtbx54A_Chan01:Motr.VBAS")

        # Ch2 - Charge buffer motor
        self.m2_relative = self.connect("WhHrdwMtbx54A_Chan02:Motr.RLV")
        self.m2_home = self.connect("WhHrdwMtbx54A_Chan02:Homf.PROC")                # Per Homing
        self.m2_reverse = self.connect("WhHrdwMtbx54A_Chan02:Homr.PROC")                
        self.m2_stop = self.connect("WhHrdwMtbx54A_Chan02:Motr.STOP")
        self.m2_done_moving = self.connect("WhHrdwMtbx54A_Chan02:Motr.DMOV")
        self.m2_speed = self.connect("WhHrdwMtbx54A_Chan02:Motr.S")
        self.m2_min_velocity = self.connect("WhHrdwMtbx54A_Chan02:Motr.VBAS")

         # Ch3 - Discharge Slider motor
        self.m3_relative = self.connect("WhHrdwMtbx54A_Chan03:Motr.RLV")
        self.m3_home = self.connect("WhHrdwMtbx54A_Chan03:Homf.PROC")  
        self.m3_absolute = self.connect("WhHrdwMtbx54A_Chan03:Motr")
        self.m3_reverse = self.connect("WhHrdwMtbx54A_Chan03:Homr.PROC")
        self.m3_stop = self.connect("WhHrdwMtbx54A_Chan03:Motr.STOP")
        self.m3_done_moving = self.connect("WhHrdwMtbx54A_Chan03:Motr.DMOV")
        self.m3_speed = self.connect("WhHrdwMtbx54A_Chan03:Motr.S")
        self.m3_min_velocity = self.connect("WhHrdwMtbx54A_Chan03:Motr.VBAS")

        # Ch4 - Discharge buffer motor
        self.m4_relative = self.connect("WhHrdwMtbx54A_Chan04:Motr.RLV")
	#self.m4_record_position = self.connect("WhHrdwMtbx54A_Chan04:Motr")
        self.m4_home = self.connect("WhHrdwMtbx54A_Chan04:Homf.PROC")
        self.m4_reverse = self.connect("WhHrdwMtbx54A_Chan04:Homr.PROC")
        self.m4_stop = self.connect("WhHrdwMtbx54A_Chan04:Motr.STOP")
        self.m4_done_moving = self.connect("WhHrdwMtbx54A_Chan04:Motr.DMOV")
        self.m4_speed = self.connect("WhHrdwMtbx54A_Chan04:Motr.S")
        self.m4_min_velocity = self.connect("WhHrdwMtbx54A_Chan04:Motr.VBAS")

        # Ch5 - Central Movement
        self.m5_relative = self.connect("WhHrdwMtbx54A_Chan05:Motr.RLV")
        self.m5_absolute = self.connect("WhHrdwMtbx54A_Chan05:Motr")
        self.m5_home= self.connect("WhHrdwMtbx54A_Chan05:Homr.PROC")                  # Per Homing
        self.m5_forward = self.connect("WhHrdwMtbx54A_Chan05:Homf.PROC")
        self.m5_stop = self.connect("WhHrdwMtbx54A_Chan05:Motr.STOP")
        self.m5_done_moving = self.connect("WhHrdwMtbx54A_Chan05:Motr.DMOV")
        self.m5_speed = self.connect("WhHrdwMtbx54A_Chan05:Motr.S")
        self.m5_min_velocity = self.connect("WhHrdwMtbx54A_Chan05:Motr.VBAS")

        # Ch6 - Coupling/Decoupling Movement
        self.m6_relative = self.connect("WhHrdwMtbx54A_Chan06:Motr.RLV")
        self.m6_absolute = self.connect("WhHrdwMtbx54A_Chan06:Motr")
        self.m6_decoupling= self.connect("WhHrdwMtbx54A_Chan06:Homr.PROC")           # Disaccoppiamento
        self.m6_coupling = self.connect("WhHrdwMtbx54A_Chan06:Homf.PROC")            # Accoppiamento
        self.m6_stop = self.connect("WhHrdwMtbx54A_Chan06:Motr.STOP")
        self.m6_done_moving = self.connect("WhHrdwMtbx54A_Chan06:Motr.DMOV")
        self.m6_speed = self.connect("WhHrdwMtbx54A_Chan06:Motr.S")
        self.m6_min_velocity = self.connect("WhHrdwMtbx54A_Chan06:Motr.VBAS")

 

        # PV - Locks
        
        # Solenoid_Top : for Charging Central
        self.lock1_insert  = self.connect("FeExprIris01A_Lock01:Insr")
        self.lock1_extract = self.connect("FeExprIris01A_Lock01:Extr")

        # Solenoid_bottom: for Discharging Central
        self.lock2_insert  = self.connect("FeExprIris01A_Lock02:Insr")
        self.lock2_extract = self.connect("FeExprIris01A_Lock02:Extr")


        # Procedure PV
        self.enable_fsm = self.connect("FeExprIris01A_Proc01:Enab")    
        self.restart_fsm = self.connect("FeExprIris01A_Proc02:Enab")                 # Restart il processo una volta finito o in caso di errore       
        self.align_disk1 = self.connect("FeExprIris01A_Proc04:Enab")                 # Allineamento Targets per la fase di "Irraggiamento"
        self.align_disk2 = self.connect("FeExprIris01A_Proc05:Enab")
        self.align_disk3 = self.connect("FeExprIris01A_Proc06:Enab")
        self.finish = self.connect("FeExprIris01A_Proc03:Enab")                      # Per terminare la fase di "Deposizione" ("Irradiation_state")
        self.coupling = self.connect("FeExprIris01A_Proc15:Enab")
        self.decoupling = self.connect("FeExprIris01A_Proc16:Enab")
        self.bufferpos = self.connect("FeExprIris01A_Proc10:Enab")

        self.state_0 = self.connect("FeExprIris01A_Proc07:Enab")
        self.state_1 = self.connect("FeExprIris01A_Proc08:Enab")                      # Stati
        self.state_2 = self.connect("FeExprIris01A_Proc09:Enab")
        #self.state_3 = self.connect("FeExprIris01A_Proc10:Enab")
        self.state_4 = self.connect("FeExprIris01A_Proc11:Enab")
        self.state_5 = self.connect("FeExprIris01A_Proc12:Enab")
        self.state_6 = self.connect("FeExprIris01A_Proc13:Enab")
        self.state_7 = self.connect("FeExprIris01A_Proc14:Enab")


        
        # START PROCEDURE
        self.gotoState('idle')

################################################################################
#                                 Charge_Slider_state                                       
################################################################################

    def idle_eval(self):
        if self.state_1.rising():
            self.gotoState(Charge_Slider_state2)

    def idle_error_eval(self):
        pass

    
# s1: Posizionamento dello slider "Charge_Slider" , Motore: m1 

# Segnalazione interfaccia
    def Charge_Slider_state_entry(self):
        # self.state_1.put(1)                                                           # Segnalazione interfaccia                                             
        self.tmrSet('moveTimeout7_1', 10)                                             # Set a timer of 30s
        
    def Charge_Slider_state_eval(self):
        if self.state_1.putCompleting():            
            self.gotoState("Charge_Slider_state2")                                    # Torna nello stato principale: "idle_state"

        elif self.tmrExpiring("moveTimeout7_1"):                                  
            self.state_1.put(4)                                                       # Condizione non prevista (segnala errore nell'interfaccia)
            if self.state_1.putCompleting():
               self.gotoState("idle_error") 

    def  Charge_Slider_state_exit(self):
         pass 
    
# movimentazione motore
    def Charge_Slider_state2_entry(self):
        self.m1_absolute.put(self.exact_charge_slider_steps)                        # Posizionamento Charge Slider                                                      
        self.logI("\tStarting Charge_Slider movement\t")
        self.tmrSet('moveTimeout7', 30)                                             # Set a timer of 30s
        
    def Charge_Slider_state2_eval(self):
        if self.m1_absolute.putCompleting():            
            self.logI("\t> - Charge Slider out - <")
            self.gotoState("idle_state")                                            # Torna nello stato principale: "idle_state"

        elif self.tmrExpiring("moveTimeout7"):                                      # Timer expired event
            self.logI("\n<  - - - !! ERROR: Charge_Slider movement - - -  >\n")
            self.state_1.put(4)                                                     # Condizione non prevista (segnala errore nell'interfaccia)
            if self.state_1.putCompleting():
               self.gotoState("idle_error") 

    def  Charge_Slider_state2_exit(self):
        self.s1 = 1                                                                 # Primo stato completato      



################################################################################
#                             Charge_state                                       
################################################################################     

# s2: Carica Charge Buffer e Central, Motore: m2 e m5, Attuatore: "lock2"

# - - -  Carica 1^ Target: Cnetral Movement, m5 - - - - - - - - - - - - - - - - - - - - -

# Segnalazione interfaccia
    def Charge_Central_state_entry(self):  
        self.state_2.put(1)
        self.tmrSet('moveTimeout8_1', 10)                                              # Set a timer of 30s

    def Charge_Central_state_eval(self):
        if self.state_2.putCompleting():
            self.gotoState("Charge_Central1_state")   

        elif self.tmrExpiring('moveTimeout8_1'):                                       # Timer expired event
            self.state_2.put(4)
            self.gotoState("idle_error")  

    def  Charge_Central_state_exit(self):
         time.sleep(5)  
    
# Estrazione attuatore
    def Charge_Central1_state_entry(self):  
        self.lock2_extract.put(1)                                                                                          # Estrae Solenoid_Top (1)
        self.tmrSet('moveTimeout8_1', 10)                                              

    def Charge_Central1_state_eval(self):
        if self.lock2_extract.putCompleting():
            self.gotoState("Charge_Central_state1")   

        elif self.tmrExpiring('moveTimeout8_1'):                                                                       
            self.state_2.put(4)
            self.gotoState("idle_error")  

    def  Charge_Central1_state_exit(self):
         pass  

# Movimentazione motore
    def Charge_Central_state1_entry(self):  
        self.m5_absolute.put(self.exact_1_charge_central_steps + self.relative_correction)                                 # 1^ Movimentazione: Carica primo e secondo Target
        self.tmrSet('moveTimeout8', 30)                                               
        self.logI("\tStarting Central movement - 1^\t")     

    def Charge_Central_state1_eval(self):
        if self.m5_absolute.putCompleting():          
            self.gotoState("Charge_state")   

        elif self.tmrExpiring('moveTimeout8'):                                        
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Charge_Central_state1_exit(self):
         pass   

# Carica 1^, 2^ Target e Scarica 1^ Target: Charge Buffer, m2 - - - - - - - - - - - - - - - - - - - - -
    def Charge_state_entry(self): 
        if self.n_restart%2 == 0:  # se pari
            self.m2_reverse.put(1)                                                                                           # Per la carica di 1 compressa
        else:                      # se dispari
            self.m2_home.put(1)

        self.tmrSet('moveTimeout10', 30)                                             
        self.logI("\tStarting Charge Buffer movement\t")

    def Charge_state_eval(self):
        if self.m2_done_moving.rising():            
            self.logI("1^ Target in \t")
            self.gotoState("Correction_CM")

        elif self.tmrExpiring("moveTimeout10"):                                       
            self.state_2.put(4) 
            self.logI("\t<  - - - !! ERROR: Charge Buffer movement - - -  >\t")
            self.gotoState("idle_error")  

    def  Charge_state_exit(self):
         pass  

# Correzione Central Movement
    def Correction_CM_entry(self):     
        self.m5_absolute.put(self.exact_1_charge_central_steps - self.relative_correction)                                  # 1^ Correzione posizione central
        self.tmrSet('moveTimeout9_1', 30)                                              
        self.logI("\tStarting Central movement: 1^Correction \t")     

    def Correction_CM_eval(self):
        if self.m5_absolute.putCompleting():          
            self.gotoState("Insert_Top1_1")   

        elif self.tmrExpiring('moveTimeout9_1'):                                      
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^Correction movement - - -  >\t")
            self.gotoState("idle_error")  

    def  Correction_CM_exit(self):
         pass 
    
# Insert solenoid top1 - 1
    def Insert_Top1_1_entry(self):  
        self.lock2_insert.put(1)                                                                                             # "Disinserisce" Solenoid_Top (1)  
        self.tmrSet('moveTimeout9', 10)                                               

    def Insert_Top1_1_eval(self):
        if self.lock2_insert.putCompleting():                                                         
            self.gotoState("Extract_lock_state2")   

        elif self.tmrExpiring('moveTimeout9'):                                         
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ movement - - -  >\t")
            self.gotoState("idle_error")  

    def  Insert_Top1_1_exit(self):
         time.sleep(2) 


# - - - Carica 2^ Target: Central Movement, m5 - - - - - - - - - - - - - - - - - - - - -

# Estrazione attuatore
    def Extract_lock_state2_entry(self):  
        self.lock2_extract.put(1)                                                                                        # Estrae Solenoid_Top (2)
        self.tmrSet('moveTimeout11_1', 10)                                             

    def Extract_lock_state2_eval(self):
        if self.lock2_extract.putCompleting():
            self.gotoState("Charge_Central_state2")   

        elif self.tmrExpiring('moveTimeout11_1'):                                       
            self.state_2.put(4)
            self.gotoState("idle_error")  

    def  Extract_lock_state2_exit(self):
         pass  
    
# Movimentazione motore Central Movement
    def Charge_Central_state2_entry(self):
        self.m5_absolute.put(self.exact_2_charge_central_steps + self.relative_correction)                                # 2^ Movimentazione: Carica secondo Target
        self.tmrSet('moveTimeout11', 30)                                                
        self.logI("\tStarting Central movement - 3^\t")     

    def Charge_Central_state2_eval(self):
        if self.m5_absolute.putCompleting():    
            self.gotoState("Charge_Buffer2_state")   

        elif self.tmrExpiring("moveTimeout11"):                                         
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 2^ movement - - -  >")
            self.gotoState("idle_error")  

    def  Charge_Central_state2_exit(self):
         pass  


# Scarica 2^ Target, Carica 3^ Target: Charge Buffer, m2  - - - - - - - - - - - - - - - - - - - - - - 
    def Charge_Buffer2_state_entry(self):      
        if self.n_restart%2 == 0: # se pari
            self.m2_home.put(1)                                                    
        else:                      # se dispari
            self.m2_reverse.put(1)                                                                                        # Per la carica della 2^ compressa e scarica del 1^

        self.tmrSet('moveTimeout13', 30)                                              
        self.logI("\tStarting Charge Buffer movement\t")

    def Charge_Buffer2_state_eval(self):
        if self.m2_done_moving.rising():                  
            self.logI("\t2^ - Target in\t")
            self.gotoState("Correction_CM2") 

        elif self.tmrExpiring("moveTimeout13"):                                       
            self.state_2.put(4) 
            self.logI("\t<  - - - !! ERROR: Charge Buffer movement - - -  >")
            self.gotoState("idle_error") 

    def  Charge_Buffer2_state_exit(self):
         pass

# Correzione Central Movement
    def Correction_CM2_entry(self):     
        self.m5_absolute.put(self.exact_2_charge_central_steps - self.relative_correction)                                 # 2^ Correzione
        self.tmrSet('moveTimeout13', 30)                                               
        self.logI("\tStarting Central movement: 2^ Correction\t")     

    def Correction_CM2_eval(self):
        if self.m5_absolute.putCompleting():          
            self.gotoState("Insert_Top1_2")   

        elif self.tmrExpiring('moveTimeout13'):                                        
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 2^ correction movement - - -  >\t")
            self.gotoState("idle_error")  

    def  Correction_CM2_exit(self):
         pass 
    
# Insert solenoid top1 - 2
    def Insert_Top1_2_entry(self):  
        self.lock2_insert.put(1)                                                                                           # "Disinserisce" Solenoid_Top (2)  
        self.tmrSet('moveTimeout13_1', 10)                                                

    def Insert_Top1_2_eval(self):
        if self.lock2_insert.putCompleting():                                                         
            self.gotoState("Extract_lock_state3")   

        elif self.tmrExpiring('moveTimeout13_1'):                                     
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 1^ movement - - -  >\t")
            self.gotoState("idle_error")  

    def  Insert_Top1_2_exit(self):
         time.sleep(2) 

# - - - Carica 3^ Target: Central Movement, m5 - - - - - - - - - - - - - - - - - - - - -

# Estrazione attuatore
    def Extract_lock_state3_entry(self):  
        self.lock2_extract.put(1)                                                                                          # Estrae Solenoid_Top (2)
        self.tmrSet('moveTimeout13_2', 10)                                              

    def Extract_lock_state3_eval(self):
        if self.lock2_extract.putCompleting():
            self.gotoState("Charge_Central_state3")   

        elif self.tmrExpiring('moveTimeout13_2'):                                      
            self.state_2.put(4)
            self.gotoState("idle_error")  

    def  Extract_lock_state3_exit(self):
         pass  
    
# Movimentazione Central Movement  
    def Charge_Central_state3_entry(self):
        self.m5_absolute.put(self.exact_3_charge_central_steps + self.relative_correction)                                # 3^ Movimentazione: Carica terzo Target
        self.tmrSet('moveTimeout14', 30)                                              
        self.logI("\tStarting Central movement - 3^\t")     

    def Charge_Central_state3_eval(self):
        if self.m5_absolute.putCompleting():          
            self.logI("\t3^ - Target in\t")
            self.gotoState("Charge_Buffer3_state")                                  

        elif self.tmrExpiring("moveTimeout14"):                                      
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 3^ movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Charge_Central_state3_exit(self):
         pass 

# Scarica 3^ e ultimo Target: Charge Buffer, m2 - - - - - - - - - - - - - - - - - - - - - - 
    def Charge_Buffer3_state_entry(self):     
        if self.n_restart%2 == 0:     # se pari
            self.m2_reverse.put(1)                                                    
        else:                         # se dispari
            self.m2_home.put(1)

        self.tmrSet('moveTimeout16', 50)                                              
        self.logI("\tStarting Charge Buffer movement\t")

    def Charge_Buffer3_state_eval(self):
        if self.m2_done_moving.rising():            
            self.logI("\t> - Charge_Buffer Discharged - <\t")
            self.gotoState("Correction_CM3") 

        elif self.tmrExpiring("moveTimeout16"):                                   
            self.state_2.put(4) 
            self.logI("\t<  - - - !! ERROR: Charge Buffer movement - - -  >\t")
            self.gotoState("idle_error") 

    def  Charge_Buffer3_state_exit(self):
           pass

# Correzione Central Movement
    def Correction_CM3_entry(self):     
        self.m5_absolute.put(self.exact_3_charge_central_steps - 1028)                                 # 3^ Correzione                     
        self.tmrSet('moveTimeout15_1', 30)                                       
        self.logI("Starting Central movement: 3^ Correction\t")     

    def Correction_CM3_eval(self):
        if self.m5_absolute.putCompleting():          
            self.gotoState("Insert_Top1_3")   

        elif self.tmrExpiring('moveTimeout15_1'):                                     
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 3^correction movement - - -  >\t")
            self.gotoState("idle_error")  

    def  Correction_CM3_exit(self):
         pass 
    
# Insert solenoid top1 - 3 
    def Insert_Top1_3_entry(self):  
        self.lock2_insert.put(1)                                                                                            # "Disinserisce" Solenoid_Top (1)  
        self.tmrSet('moveTimeout15', 10)                                               

    def Insert_Top1_3_eval(self):
        if self.lock2_insert.putCompleting():                                                         
            self.gotoState("avanti_tutta")   

        elif self.tmrExpiring('moveTimeout15'):                                          
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : inserting lock 2 - - -  >\t")
            self.gotoState("idle_error")  

    def  Insert_Top1_3_exit(self):
         pass 


# avanti tutta
    def avanti_tutta_entry(self):     
        self.m5_absolute.put(self.exact_3_charge_central_steps + 64*20)                                 # 3^ Correzione                     
        self.tmrSet('moveTimeout15_1_!', 30)                                       
        self.logI("Starting Central movement: 3^ Correction avanti tutta\t")     

    def avanti_tutta_eval(self):
        if self.m5_absolute.putCompleting():          
            self.gotoState("Charge_Central_state4")   

        elif self.tmrExpiring('moveTimeout15_1_1'):                                     
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Central movement : during 3^correction movement - - -  >\t")
            self.gotoState("idle_error")  

    def  avanti_tutta_exit(self):
         pass 

# Homing Central: Cnetral Movement, m5 - - - - - - - - - - - - - - - - - - - - -
    def Charge_Central_state4_entry(self):
        self.m5_home.put(1)
        self.tmrSet('moveTimeout17', 30)             
        self.logI("\t< Homing Cental Movement >\t")

    def Charge_Central_state4_eval(self):
        if self.m5_done_moving.rising():            
            self.gotoState("Charge_Central_state5")                 

        elif self.tmrExpiring("moveTimeout17"):
            self.state_2.put(4)
            self.logI("\tERROR\t")
            self.gotoState("idle_error")

    def Charge_Central_state4_exit(self):
        pass

# "Homing" Charge_Slider, m1 - - - - - - - - - - - - - - - - - - - - -
    def Charge_Central_state5_entry(self):       
        self.m1_absolute.put(0)                                                          # Preparazione per il vuoto: Homing Charge_Slider
        self.tmrSet('moveTimeout18', 30)                                             
        self.logI("\tStarting Central movement")     

    def Charge_Central_state5_eval(self):
        if self.m1_absolute.putCompleting():          
            self.gotoState("last_charge_state")   

        elif self.tmrExpiring("moveTimeout18"):                                     
            self.state_2.put(4)
            self.logI("\t<  - - - !! ERROR: Charge Slider movement - - -  >\n")
            self.gotoState("idle_error")  

    def  Charge_Central_state5_exit(self):
         pass

# Fine processo di carica 
    def last_charge_state_entry(self):       
        self.state_2.put(2)
        self.tmrSet('moveTimeout18_1', 10) 

    def last_charge_state_eval(self):
        if self.state_2.putCompleting():          
            self.logI("\t >  - - - Iris is ready for the vacuum - - - <")
            self.s2 = 1 # Fine stato di carica
            self.gotoState("idle_state")   

        elif self.tmrExpiring("moveTimeout18_1"):                                      
            self.state_2.put(4)
            self.gotoState("idle_error")  

    def  last_charge_state_exit(self):
         pass
                                     
