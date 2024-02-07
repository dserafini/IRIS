
import time
from smlib import fsmBase

class prepare_fsm(fsmBase):
    def __init__(self, name, *args, **kwargs):
        super(prepare_fsm, self).__init__(name, **kwargs)

        
        # ALIGNMENT - - - - - - - - - - - - - - - - - - - - - - - -
        # Charge_Slider positions
        self.exact_charge_slider_steps = 6656     # 6592+64
        self.exact_discharge_slider_steps = -6720 # -6336+64
        self.exact_longitudinal_axis_steps = 128

        # Charge_Central_Movement position
        self.exact_1_charge_central_steps = self.connect("FeExprIris01A_Moto02:Poin")   # 7872
        self.exact_2_charge_central_steps = self.connect("FeExprIris01A_Moto03:Poin") # 16640
        self.exact_3_charge_central_steps = self.connect("FeExprIris01A_Moto04:Poin") 

        # Discharge Central Movement position
        self.exact_1_discharge_central_steps = 4096
        self.exact_2_discharge_central_steps = 13248-64
        self.exact_3_discharge_central_steps = 21568 # 21568

        # Charge Central Movement Correction
        self.relative_correction = 64  # correzione di uno step
        
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
        self.state_8 = self.connect("FeExprIris01A_Proc15:Enab")


        
        # START PROCEDURE
        self.gotoState('check_connections')



################################################################################
#                               CONNECTION PV'S
################################################################################

    def idle_eval(self):
        if self.state_8.rising():
            self.gotoState("check_connections")

    def idle_error_eval(self):
        pass

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
	    # Controlo stati 
        self.s1 = 0
        self.s2 = 0
        #self.s3 = 0                                                                  #Inutilizzato 
        self.s4 = 0
        self.s5 = 0
        self.s6 = 0
        self.s7 = 0
	    # Irradiation_state: Alignment Targets
        self.d1 = 0  
        self.d2 = 0  
        self.d3 = 0
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
            self.gotoState("motor_settings_state_m5_2")
        elif self.tmrExpiring("moveTimeout05_2"):                                    
            self.logI("\t... ERRORE: motor_settings_state ...")
            self.gotoState("idle_error") 
		

    def motor_settings_state_m5_2_entry(self):
# M5
        self.exact_1_charge_central_steps.put(8064)  # 7872
        self.exact_2_charge_central_steps.put(16704) # 16640
        self.exact_3_charge_central_steps.put(24960) 

    def motor_settings_state_m5_2_eval(self):
        if self.exact_1_charge_central_steps.putCompleting() and
           self.exact_2_charge_central_steps.putCompleting() and
           self.exact_3_charge_central_steps.putCompleting():
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
        self.state_8.put(0)
        self.tmrSet('moveTimeout020', 5)

    def interface_state_7_eval(self):
        if self.state_7.putCompleting() and
           self.state_8.putCompleting():
            self.logI("\tAll PV and Motor Inizialized ... ")
            self.gotoState("idle")
        elif self.tmrExpiring("moveTimeout020"):                                    
            self.logI("\t... ERRORE: interface_state_7 ...")
            self.gotoState("idle_error")
		
