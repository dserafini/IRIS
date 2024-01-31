
import time
from smlib import fsmBase

class discharge2_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(discharge2_fsm, self).__init__(name, **kwargs)

        
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
#                           Discharge_Central_state                                      
################################################################################   

# s6 : Carica "Central Movement", Motore: m5 , Attuatore: "lock1" 
  
    def idle_eval(self):
        if self.state_6.rising():
            self.gotoState("Extract_lock1_1")   
		
    def idle_error_eval(self):
        pass



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
        self.state_6.put(0)                                                             
        self.tmrSet('moveTimeout31_1', 10)                                              

    def final_discharge_state_eval(self):
        if self.state_6.putCompleting():                                                                 
            self.gotoState("idle")    
 
        elif self.tmrExpiring("moveTimeout31_1"):                                       
            self.state_6.put(4)
            self.gotoState("idle_error")  

    def  final_discharge_state_exit(self):
         self.s6 = 1                                                                                                                               # Sesto stato completato
