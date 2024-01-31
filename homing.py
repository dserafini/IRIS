import time
from smlib import fsmBase

class homing_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(homing_fsm, self).__init__(name, **kwargs)

        
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
#                               HOMING MOTORS
################################################################################

    def idle_eval(self):
        pass

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
