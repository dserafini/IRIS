import time
from smlib import fsmBase

class irradiation_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(irradiation_fsm, self).__init__(name, **kwargs)

        
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
#                             Irradiation_state                                     
################################################################################    
    def idle_eval(self):
        pass

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
