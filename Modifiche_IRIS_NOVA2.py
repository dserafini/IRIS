
import time
from smlib import fsmBase

class IRIS_FSM(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(IRIS_FSM, self).__init__(name, **kwargs)

        
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
        self.gotoState('check_connections')



################################################################################
#                               CONNECTION PV'S
################################################################################

    def check_connections_entry(self):
        self.logI("\tChecking PVs connections...")
        self.tmrSet('moveTimeout00', 5)

    def check_connections_eval(self):
        # Ricomincia dall'inizio
        if self.isIoInitialized():
            self.logI("... PVs connections are OK!")
            self.gotoState("motor_settings_state")
        elif self.tmrExpiring("moveTimeout00"):                                    
            self.logI("\t... PVs connections Checking Failed ...")
            self.gotoState("idle_error") 

    def check_connections_exit(self):
        pass

################################################################################
#                               MOTOR SETTINGS STATE
################################################################################
    def motor_settings_state_entry(self):
        # Iniziallizzazione Variabili interne ai valori di default
        self.s1 = 0
        self.s2 = 0
        #self.s3 = 0                                                                  #Inutilizzato 
        self.s4 = 0
        self.s5 = 0
        self.s6 = 0
        self.s7 = 0
        self.d1 = 0  
        self.d2 = 0  
        self.d3 = 0
        self.e  = 0  
        self.coupling_push_botton = 0
        self.decoupling_push_botton = 0
# M1
        self.m1_min_velocity.put(1280) 
        self.tmrSet('moveTimeout01', 5)

    def motor_settings_state_eval(self):
        if self.m1_min_velocity.putCompleting():
            self.gotoState("motor_settings_state_1")
        elif self.tmrExpiring("moveTimeout01"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_1_entry(self):
# M1
        self.m1_speed.put(0.2) 
        self.tmrSet('moveTimeout01_1', 5)

    def motor_settings_state_1_eval(self):
        if self.m1_speed.putCompleting():
            self.gotoState("motor_settings_state_m2")
        elif self.tmrExpiring("moveTimeout01_1"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m2_entry(self):
# M2
        self.m2_min_velocity.put(1280)   
        self.tmrSet('moveTimeout02_1', 5)

    def motor_settings_state_m2_eval(self):
        if self.m2_min_velocity.putCompleting():
            self.gotoState("motor_settings_state_m2_1")
        elif self.tmrExpiring("moveTimeout02_1"):                                    
            self.logI("\t... ERRORE: motor_settings_state_m2...")
            self.gotoState("idle_error") 

    def motor_settings_state_m2_1_entry(self):
# M2
        self.m2_speed.put(0.2) 
        self.tmrSet('moveTimeout02_2', 5)

    def motor_settings_state_m2_1_eval(self):
        if self.m2_speed.putCompleting():
            self.gotoState("motor_settings_state_m3")
        elif self.tmrExpiring("moveTimeout02_2"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m3_entry(self):
# M3
        self.m3_min_velocity.put(1280)   
        self.tmrSet('moveTimeout03_1', 5)

    def motor_settings_state_m3_eval(self):
        if self.m3_min_velocity.putCompleting():
            self.gotoState("motor_settings_state_m3_1")
        elif self.tmrExpiring("moveTimeout03_1"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m3_1_entry(self):
# M3
        self.m3_speed.put(0.1) 
        self.tmrSet('moveTimeout03_2', 5)

    def motor_settings_state_m3_1_eval(self):
        if self.m3_speed.putCompleting():
            self.gotoState("motor_settings_state_m4")
        elif self.tmrExpiring("moveTimeout03_2"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m4_entry(self):
# M4
        self.m4_min_velocity.put(1280)   
        self.tmrSet('moveTimeout04_1', 5)

    def motor_settings_state_m4_eval(self):
        if self.m4_min_velocity.putCompleting():
            self.gotoState("motor_settings_state_m4_1")
        elif self.tmrExpiring("moveTimeout04_1"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m4_1_entry(self):
# M4
        self.m4_speed.put(0.2) 
        self.tmrSet('moveTimeout04_2', 5)

    def motor_settings_state_m4_1_eval(self):
        if self.m4_speed.putCompleting():
            self.gotoState("motor_settings_state_m5")
        elif self.tmrExpiring("moveTimeout04_2"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m5_entry(self):
# M5
        self.m5_min_velocity.put(1280)   
        self.tmrSet('moveTimeout05_1', 5)

    def motor_settings_state_m5_eval(self):
        if self.m5_min_velocity.putCompleting():
            self.gotoState("motor_settings_state_m5_1")
        elif self.tmrExpiring("moveTimeout05_1"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m5_1_entry(self):
# M5
        self.m5_speed.put(0.2) 
        self.tmrSet('moveTimeout05_2', 5)

    def motor_settings_state_m5_1_eval(self):
        if self.m5_speed.putCompleting():
            self.gotoState("motor_settings_state_m6")
        elif self.tmrExpiring("moveTimeout05_2"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m6_entry(self):
# M6
        self.m6_min_velocity.put(128)   
        self.tmrSet('moveTimeout06_1', 5)

    def motor_settings_state_m6_eval(self):
        if self.m6_min_velocity.putCompleting():
            self.gotoState("motor_settings_state_m6_1")
        elif self.tmrExpiring("moveTimeout06_1"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

    def motor_settings_state_m6_1_entry(self):
# M6
        self.m6_speed.put(0.640) 
        self.tmrSet('moveTimeout06_2', 5)

    def motor_settings_state_m6_1_eval(self):
        if self.m6_speed.putCompleting():
            self.gotoState("interface_1")
        elif self.tmrExpiring("moveTimeout06_2"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 

# ---  Reset interfaccia   ---- 

    def interface_1_entry(self):
# restart_fsm
        self.restart_fsm.put(0)   
        self.tmrSet('moveTimeout07', 5)

    def interface_1_eval(self):
        if self.restart_fsm.putCompleting():
            self.gotoState("interface_2")
        elif self.tmrExpiring("moveTimeout07"):                                    
            self.logI("\t... ERRORE: interface_1 ...")
            self.gotoState("idle_error") 

    def interface_2_entry(self):
# enable_fsm
        self.enable_fsm.put(0) 
        self.tmrSet('moveTimeout08', 5)

    def interface_2_eval(self):
        if self.enable_fsm.putCompleting():
            self.gotoState("interface_3")
        elif self.tmrExpiring("moveTimeout08"):                                    
            self.logI("\t... ERRORE: interface_2 ...")
            self.gotoState("idle_error") 

    def interface_3_entry(self):
# finish
        self.finish.put(0)   
        self.tmrSet('moveTimeout09', 5)

    def interface_3_eval(self):
        if self.finish.putCompleting():
            self.gotoState("interface_d1")
        elif self.tmrExpiring("moveTimeout09"):                                    
            self.logI("\t... ERRORE: interface_3 ...")
            self.gotoState("idle_error") 

# Reset Disk  - - -

    def interface_d1_entry(self):
# align_disk1
        self.align_disk1.put(0) 
        self.tmrSet('moveTimeout010', 5)

    def interface_d1_eval(self):
        if self.align_disk1.putCompleting():
            self.gotoState("interface_d2")
        elif self.tmrExpiring("moveTimeout010"):                                    
            self.logI("\t... ERRORE: interface_d1 ...")
            self.gotoState("idle_error") 

    def interface_d2_entry(self):
# align_disk2
        self.align_disk2.put(0) 
        self.tmrSet('moveTimeout011', 5)

    def interface_d2_eval(self):
        if self.align_disk2.putCompleting():
            self.gotoState("interface_d3")
        elif self.tmrExpiring("moveTimeout011"):                                    
            self.logI("\t... ERRORE: interface_d2 ...")
            self.gotoState("idle_error") 

    def interface_d3_entry(self):
# align_disk3
        self.align_disk3.put(0) 
        self.tmrSet('moveTimeout012', 5)

    def interface_d3_eval(self):
        if self.align_disk3.putCompleting():
            self.gotoState("interface_state_0")
        elif self.tmrExpiring("moveTimeout012"):                                    
            self.logI("\t... ERRORE: interface_d3 ...")
            self.gotoState("idle_error") 

# Reset Led  - - -

    def interface_state_0_entry(self):
# state_0
        self.state_0.put(0)
        self.tmrSet('moveTimeout013', 5)

    def interface_state_0_eval(self):
        if self.state_0.putCompleting():
            self.gotoState("interface_state_1")
        elif self.tmrExpiring("moveTimeout013"):                                    
            self.logI("\t... ERRORE: interface_state_0 ...")
            self.gotoState("idle_error") 

    def interface_state_1_entry(self):
# state_1
        self.state_1.put(self.s1) 
        self.tmrSet('moveTimeout014', 5)

    def interface_state_1_eval(self):
        if self.state_1.putCompleting():
            self.gotoState("interface_state_2")
        elif self.tmrExpiring("moveTimeout014"):                                    
            self.logI("\t... ERRORE: interface_state_1 ...")
            self.gotoState("idle_error") 

    def interface_state_2_entry(self):
# state_2
        self.state_2.put(self.s2) 
        self.tmrSet('moveTimeout015', 5)

    def interface_state_2_eval(self):
        if self.state_2.putCompleting():
            self.gotoState("interface_state_4")
        elif self.tmrExpiring("moveTimeout015"):                                    
            self.logI("\t... ERRORE: interface_state_2 ...")
            self.gotoState("idle_error") 

    #def interface_state_3_entry(self):
# state_3
        #self.state_3.put(self.s3)                                                      # inutilizzato (attenzione: non viene controllata la sua connessione ) 
        #self.tmrSet('moveTimeout016', 5)

    #def interface_state_3_eval(self):
        #if self.state_3.putCompleting():
            #self.gotoState("interface_state_4")
        #elif self.tmrExpiring("moveTimeout016"):                                    
            #self.logI("\t... ERRORE: interface_state_3 ...")
            #self.gotoState("idle_error") 

    def interface_state_4_entry(self):
# state_4
        self.state_4.put(self.s4) 
        self.tmrSet('moveTimeout017', 5)

    def interface_state_4_eval(self):
        if self.state_4.putCompleting():
            self.gotoState("interface_state_5")
        elif self.tmrExpiring("moveTimeout017"):                                    
            self.logI("\t... ERRORE: interface_state_4 ...")
            self.gotoState("idle_error") 

    def interface_state_5_entry(self):
# state_5
        self.state_5.put(self.s5) 
        self.tmrSet('moveTimeout018', 5)

    def interface_state_5_eval(self):
        if self.state_5.putCompleting():
            self.gotoState("interface_state_6")
        elif self.tmrExpiring("moveTimeout018"):                                    
            self.logI("\t... ERRORE: interface_state_5 ...")
            self.gotoState("idle_error") 

    def interface_state_6_entry(self):
# state_6
        self.state_6.put(self.s6) 
        self.tmrSet('moveTimeout019', 5)

    def interface_state_6_eval(self):
        if self.state_6.putCompleting():
            self.gotoState("interface_state_7")
        elif self.tmrExpiring("moveTimeout019"):                                    
            self.logI("\t... ERRORE: interface_state_6 ...")
            self.gotoState("idle_error") 

    def interface_state_7_entry(self):
# state_7
        self.state_7.put(self.s7) 
        self.tmrSet('moveTimeout020', 5)

    def interface_state_7_eval(self):
        if self.state_7.putCompleting():
            self.logI("\tAll PV and Motor Inizialized ... ")
            self.gotoState("homing")
        elif self.tmrExpiring("moveTimeout020"):                                    
            self.logI("\t... ERRORE: interface_state_7 ...")
            self.gotoState("idle_error") 


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
            

################################################################################
#                                 Charge_Slider_state                                       
################################################################################    

# s1: Posizionamento dello slider "Charge_Slider" , Motore: m1 

# Segnalazione interfaccia
    def Charge_Slider_state_entry(self):
        self.state_1.put(1)                                                           # Segnalazione interfaccia                                             
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
