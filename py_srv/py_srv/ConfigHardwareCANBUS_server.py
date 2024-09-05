#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from exo_interfaces.srv import ConfigHardwareTrajectory
from std_msgs.msg import UInt16, Float64
from collections import deque
from itertools import product #para poner dos for seguidos
import numpy as np
from numpy import convolve as np_convolve
import struct
import time
import yaml
import pandas as pd
import sys
import os
import can

# EXPLICACIÓN:
# Se ha creado un server que recibe los parametros del hardware para configurar la conexión con el canbus y la creación de los publishers.
# Se ha condicionado a que no se vean la información de los publishers.
# Se ha comprobado la frecuencia a la que se publican los publishers y se encuentra entre 0.003-0.006, nuestra frecuencia de callback es de 0.002. Comanado: ros2 topic hz /PUB_left_hip_gauge
# A veces a aparece una id=4 vacia, esta puede ser que se deba a que el programa se esta ejecutando más despacio de lo que esta configurado, y a veces aparece un mensaje vacio. Se ha modificado el codigo para eliminar este mensaje de la terminal.

# IMP:Realizando estas modificaione se ha comprobado el filtro tanto de la galga como el del potenciometro y estan bien. El valor de la galga anda desplazado un 0.2 que es factible, mientras que el potenciometro...

# IMP: Lo siguiente sería calibrar el exosqueleto, todos los sensores con cada una de las articulaciones. En este codigo se han creado los publisher tanto en crudo, filtrado y final para poder comprobar en la grafica de rqt su comportamiento.


msg = UInt16()

"""Global variables"""
joints = ["right_knee", "left_knee", "right_hip", "left_hip"]
#hw_names = ["Potentiometer", "Gauge", "FSR1", "FSR2"]
hw_names = ["potentiometer", "gauge", "fsr1", "fsr2"]



class CANbus(object):
    def __init__(self, channel, hardware_config_service, bustype='socketcan'):    
        self.hw_config = hardware_config_service
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1000000)
        self.buffer = can.BufferedReader()
        self.verbose = self.hw_config.verbose
        #print("DEntro canbus")
    def __del__(self):
        self.bus.shutdown()

    """ Send the following data frame to receive a data package from the sensor acquisition board:
        Message type: Standard (11-bit identifier)
        Message identifier (ID): 68
        Pack data: 0
    """
    def send_command(self):
        id = 68
        msg = can.Message(arbitration_id=id, is_extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])   # 11-bit identifier (not-extended)
        
        if self.verbose == True:
            print("\n\n","\033[93m[INFO] Sending initialization ID for empty message ----> ID: {}\033[0m".format(id))
            print("Sending msg from send_command of can_rpi: {}".format(msg))
        self.bus.send(msg)

    """ Keep hearing CAN port until timeout is reached. This function should be
        used after sending the commanding data frame (previous function)
    """
    def receive_data(self):
        msg = self.bus.recv(0.001) # Timeout in seconds. None: Wait until data is received.  
        if msg is None:
            print('Timeout occurred, no message.')
            return None
        if msg is not None:
            if self.verbose == True:
                print("Message in def receive_data: {}".format(msg))
            return msg

class ConfigHardwareService(Node):

    def __init__(self):
        super().__init__('config_hardware_server')
        self.srv = self.create_service(ConfigHardwareTrajectory, 'configuration', self.hardware_configuration_callback)
        self.get_logger().info("[INFO] Ready to inicializate paramters!")
        #self.get_logger().warn("[WARN] TRYING TO CORRECT THE LINEAL REGRESION OF ALL THE SENSORS!")

        #Initializate verbose as an instance variable
        self.verbose = None
        self.elevation_port = None
        self.exo_port = None
        self.sampling_frequency = None

        # right_knee
        self.right_knee_can_id = None
        self.right_knee_potentiometer = None
        self.right_knee_gauge = None
        self.right_knee_fsr1 = None
        self.right_knee_fsr2 = None

        # left_knee
        self.left_knee_can_id = None
        self.left_knee_potentiometer = None
        self.left_knee_gauge = None
        self.left_knee_fsr1 = None
        self.left_knee_fsr2 = None

        # right_hip
        self.right_hip_can_id = None
        self.right_hip_potentiometer = None
        self.right_hip_gauge = None
        self.right_hip_fsr1 = None
        self.right_hip_fsr2 = None

        # left_hip
        self.left_hip_can_id = None
        self.left_hip_potentiometer = None
        self.left_hip_gauge = None
        self.left_hip_fsr1 = None
        self.left_hip_fsr2 = None

        # Variables for filter
        self.filter_states = [0.0]*2
        self.gauge_data  = [None] * 318
        self.init_params_()

    """Parametros de ajuste para la regresión lineal de los potenciometros según su joint_name"""
    def init_params_(self):
        # TODO: Check values
        self.pot_params_all = {'left_knee':[-0.105, 98.401],
                     'right_knee':[0.1087, -10.109],
                     'left_hip':[-0.0983, 67.598],
                     'right_hip':[-0.1061, 56.675]
                     }
        #self.pot_params = self.pot_params_all[self.joint_name]
        for joint_name in joints:
             self.pot_params = self.pot_params_all[joint_name]


    def hardware_configuration_callback(self, request, response):
        # Se definen las variables
        self.verbose = bool(request.verbose)
        self.elevation_port = str(request.elevation_port)
        self.exo_port = str(request.exo_port)
        self.sampling_frequency = request.sampling_frequency
        
        #right_knee
        self.right_knee_can_id =  int(request.right_knee_can_id)
        self.right_knee_potentiometer = bool(request.right_knee_potentiometer)
        self.right_knee_gauge = bool(request.right_knee_gauge)
        self.right_knee_fsr1 = bool(request.right_knee_fsr1)
        self.right_knee_fsr2 = bool(request.right_knee_fsr2)

        #left_knee
        self.left_knee_can_id = int(request.left_knee_can_id)
        self.left_knee_potentiometer = bool(request.left_knee_potentiometer)
        self.left_knee_gauge = bool(request.left_knee_gauge)
        self.left_knee_fsr1 = bool(request.left_knee_fsr1)
        self.left_knee_fsr2 = bool(request.left_knee_fsr2)

        #right_hip
        self.right_hip_can_id = int(request.right_hip_can_id)
        self.right_hip_potentiometer = bool(request.right_hip_potentiometer)
        self.right_hip_gauge = bool(request.right_hip_gauge)
        self.right_hip_fsr1 = bool(request.right_hip_fsr1)
        self.right_hip_fsr2 = bool(request.right_hip_fsr2)

        #left_hip
        self.left_hip_can_id = int(request.left_hip_can_id)
        self.left_hip_potentiometer = bool(request.left_hip_potentiometer)
        self.left_hip_gauge = bool(request.left_hip_gauge)
        self.left_hip_fsr1 = bool(request.left_hip_fsr1)
        self.left_hip_fsr2 = bool(request.left_hip_fsr2)

        if all(v is not None for v in (self.verbose, self.elevation_port, self.exo_port, self.sampling_frequency,
            self.right_knee_can_id, self.right_knee_potentiometer, self.right_knee_gauge, self.right_knee_fsr1,
            self.right_knee_fsr2, self.left_knee_can_id, self.left_knee_potentiometer, self.left_knee_gauge,
            self.left_knee_fsr1, self.left_knee_fsr2, self.right_hip_can_id, self.right_hip_potentiometer,
            self.right_hip_gauge, self.right_hip_fsr1, self.right_hip_fsr2, self.left_hip_can_id,
            self.left_hip_potentiometer, self.left_hip_gauge, self.left_hip_fsr1, self.left_hip_fsr2)):
            response.success = True
            if self.verbose:
                self.get_logger().info(f"Parameters of information: verbose: {self.verbose}")
                self.get_logger().info(f"Parameters of can: elevation_port: {self.elevation_port}, exo_port: {self.exo_port}")
                self.get_logger().info(f"Parameters of frequency: {self.sampling_frequency}")
                self.get_logger().info(f"RIGHT_KNEE: right_knee_can_id: {self.right_knee_can_id}, right_knee_potentiometer: {self.right_knee_potentiometer}, right_knee_gauge: {self.right_knee_gauge}, right_knee_fsr1: {self.right_knee_fsr1}, right_knee_fsr2: {self.right_knee_fsr2}")
                self.get_logger().info(f"LEFT_KNEE: left_knee_can_id: {self.left_knee_can_id}, left_knee_potentiometer: {self.left_knee_potentiometer}, left_knee_gauge: {self.left_knee_gauge}, left_knee_fsr1: {self.left_knee_fsr1}, left_knee_fsr2: {self.left_knee_fsr2}")
                self.get_logger().info(f"RIGHT_HIP:  right_hip_can_id: {self.right_hip_can_id}, right_hip_potentiometer: {self.right_hip_potentiometer}, right_hip_gauge: {self.right_hip_gauge}, right_hip_fsr1: {self.right_hip_fsr1}, right_hip_fsr2: {self.right_hip_fsr2}")
                self.get_logger().info(f"LEFT_HIP: left_hip_can_id: {self.left_hip_can_id}, left_hip_potentiometer: {self.left_hip_potentiometer}, left_hip_gauge: {self.left_hip_gauge}, left_hip_fsr1: {self.left_hip_fsr1}, left_hip_fsr2: {self.left_hip_fsr2}" ) 

            #self.get_logger().info("[INFO] INITIALIZATION OF HARDWARE CONFIGURATION FINISHED!")
            self.PUBLISHER_CANBUS_Initialization() #se condiciona que solo se creen los publishers cuando sea true los valores

        else:
            response.success = False
            self.get_logger().info("Failed to read parameters. Some parameters are missing")
        return response

    def PUBLISHER_CANBUS_Initialization(self):
        # Create Publisher and dictionary for can_id (to select joint)
        self.can_id_joint_dict = {}
        for joint_name in joints:
            #Creamos un diccionario donde relacione joint_name: can_id
            can_id = getattr(self, f"{joint_name}_can_id")
            self.can_id_joint_dict[joint_name] = can_id
            for hw_name in hw_names:
                #Si el valor obtenido de request es true se crea el publisher de esa variable
                if getattr(self, f"{joint_name}_{hw_name}"):
                    # Creamos el publisher final
                    topic_name = 'PUB_{}_{}'.format(joint_name, hw_name)
                    publisher = self.create_publisher(Float64, topic_name, 100)
                    setattr(self, f"{joint_name}_{hw_name}_publisher", publisher)
                    #Crear un publisher para comparar los valores en crudo
                    topic_name_crudo = 'PUB_{}_{}_crudo'.format(joint_name, hw_name)
                    publisher_crudo = self.create_publisher(UInt16, topic_name_crudo, 100)
                    setattr(self, f"{joint_name}_{hw_name}_publisher_crudo", publisher_crudo)
                    #Crear un publisher para comparar los valores tras el filtro
                    topic_name_filter = 'PUB_{}_{}_filter'.format(joint_name, hw_name)
                    publisher_filter = self.create_publisher(Float64, topic_name_filter, 100)
                    setattr(self, f"{joint_name}_{hw_name}_publisher_filter", publisher_filter)

        # Le damos la vuelta al diccionario: can_id: joint_name
        self.inv_joint_can_id = {v: k for k, v in self.can_id_joint_dict.items()}
        if self.verbose:
            print("[INFO] DICTIONARY (can_id: joint_name) ---->", self.can_id_joint_dict)
            print("[INFO] DICTIONARY (joint_name: can_id) ---->", self.inv_joint_can_id)

        """CAN BUS initializarion"""
        if self.exo_port == "can0" or self.exo_port == "can1":
            self.can_bus = CANbus(channel=self.exo_port, hardware_config_service=self)
            #self.get_logger().info("[INFO] INITIALIZATION OF CANBUS FINISHED!")
            """ROS initialization"""
            self.timer = self.create_timer(1 / self.sampling_frequency, self.timer_callback)
            #self.get_logger().info("[INFO] INITIALIZATION OF PUBLISHERS FINISHED!")
            self.get_logger().info("[INFO] COMMUNICATION COMPLETE! START LECTURE OF SENSORS:")
        else:
             self.get_logger().error("Failed to read parameters. The value of exo_port is incorrect")

        

    # Iterate through the joints and read their hardware configuration and CAN ID
    def timer_callback(self):
        self.can_bus.send_command()
        for i in range(len(joints)):
            msg = self.can_bus.receive_data()
            if msg is not None:
                try:
                    if msg.arbitration_id == 4:
                            continue # Ignorar mensajes con arbitration_id igual a 4 (son mensajes vacios)
                    joint_id = self.inv_joint_can_id[msg.arbitration_id]  # get joint name from joint can id

                    if self.verbose == True:
                        print("\033[95m[SELECT] SELECT ID FROM MSG!! ----> ID: {}\033[0m".format(joint_id))
                    self.get_sensor_data(msg, joint_id)
                except:
                    self.get_logger().error('[Exo] Error! Unknown id {}!'.format(msg.arbitration_id))
            else:
                self.get_logger().info('[INFO] Timeout occurred, no message.')

        # self.rate.sleep()



	# Function that unpack the message from the different sensors
    def get_sensor_data(self, msg, joint_id):
        potentiometer_msg = UInt16()
        pot_msg = Float64()
        filt_pot = Float64()
        gauge_crudo_msg = UInt16()
        gauge_msg = Float64()
        gauge_filtered = Float64()
        fsr1_msg = UInt16()
        fsr2_msg = UInt16()

        try:
            potentiometer_msg.data = struct.unpack('<H', msg.data[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
            #pot_msg.data = ((potentiometer_msg.data / 1024) * 360)  # We want the number without decimals.
            getattr(self, f"{joint_id}_potentiometer_publisher_crudo").publish(potentiometer_msg)
            pot_msg.data = self.pot2angle(self.biquad_filter(potentiometer_msg.data))
            getattr(self, f"{joint_id}_potentiometer_publisher_filter").publish(filt_pot)
            getattr(self, f"{joint_id}_potentiometer_publisher").publish(pot_msg)
            if self.verbose == True:
                self.get_logger().info('Publishing Potentiometer reading for joint {}: "{:.0f}"'.format(joint_id, pot_msg.data))
        except Exception as e:
            self.get_logger().error('[Exo] Error! Failed in the lecture of Potentiometer for {}: {}'.format(joint_id, e))

        if getattr(self, f"{joint_id}_gauge"):
            try:
                gauge_crudo_msg.data = struct.unpack('<H', msg.data[2:4])[0]
                getattr(self, f"{joint_id}_gauge_publisher_crudo").publish(gauge_crudo_msg)
                gauge_filtered.data = self.FIR_filter(float(gauge_crudo_msg.data)) 
                if gauge_filtered.data > 0.0:
                    getattr(self, f"{joint_id}_gauge_publisher_filter").publish(gauge_filtered)
                    #gauge_msg.data = 24.525 - 0.2133 * gauge_filtered.data
                    gauge_msg.data = 22.264 - 0.0427 * gauge_filtered.data
                    getattr(self, f"{joint_id}_gauge_publisher").publish(gauge_msg)
                    if self.verbose == True:
                        self.get_logger().info('Publishing Gauge reading for joint {}: "{}"'.format(joint_id, gauge_msg.data))  
                else:
                    if self.verbose == True:
                        self.get_logger().info("Waiting. FIR_FILTER incomplete.")
            except Exception as e:
                self.get_logger().error('[Exo] Error! Failed in the lecture of Gauge for {}: {}'.format(joint_id, e))
                #print("except_gauge_filtered-data: {}".format(gauge_filtered.data))
                #print("type_except_gauge_filtered-data: {}".format(type(gauge_filtered.data)))
                


        if getattr(self, f"{joint_id}_fsr1"):
            fsr1_msg.data = struct.unpack('<H', msg.data[4:6])[0]   # 5-6 bytes correspond to potentiometer reading
            self.get_logger().info('Publishing FSR1 reading for joint {}: "{}"'.format(joint_id, fsr1_msg.data))
            getattr(self, f"{joint_id}_fsr1_publisher").publish(fsr1_msg)

        if getattr(self, f"{joint_id}_fsr2"):
            fsr2_msg.data = struct.unpack('<H', msg.data[6:])[0]    # 7-8 bytes correspond to potentiometer reading
            getattr(self, f"{joint_id}_fsr2_publisher").publish(fsr2_msg)
            #self.get_logger().info('Publishing FSR2 reading for joint {}: "{}"'.format(joint_id, fsr2_msg.data))



    """Biquad filter for potentiometer readings (Exported from Simulink)"""
    def biquad_filter(self, pot):
        '''Filter parameters'''
        a0 = 0.000956278151630223
        a1 = -1.9106427906788
        a2 = 0.914467903285316
        b0 = 2.0
        den_accum = (a0*pot - a1*self.filter_states[0]) - a2*self.filter_states[1]
        filt_pot = (b0*self.filter_states[0] + den_accum) + self.filter_states[1]
        '''Updating filter states'''
        self.filter_states[1] = self.filter_states[0]
        self.filter_states[0] = den_accum
        return filt_pot

    """ Converts from potentiometer reading to angle value.
        The transference funtion depends on the joint being used."""
    def pot2angle(self, pot):
        # parameters for each joint are pre-encoded in the list for efficiency
        # the output angle can be obtained with the given formula:
        return self.pot_params[0]*pot + self.pot_params[1]

    """FIR filter for gauge readings (Values exported from Simulink)"""
    def FIR_filter(self, raw_gauge):
        ''' Data received'''
        lst = self.gauge_data
        lst = deque(lst)
        lst.rotate(1)
        lst = list(lst)
        self.gauge_data = lst
        self.gauge_data[0] = raw_gauge;
        '''Filter parameters'''
        # Verifica si la ventana está completa
        if all(value is not None for value in self.gauge_data):
            b = [-7.4029787723175851e-05 , 1.7335114024242748e-05 , 1.6714489931227413e-05 , 1.6914664698078393e-05 , 1.7833760715256158e-05 , 1.936753284143648e-05 , 2.1470175773501382e-05 , 2.40808761912665e-05 , 2.7190109935793652e-05 , 3.076222082586599e-05 , 3.4808770994987535e-05 , 3.9309720691864923e-05 , 4.4291245517772101e-05 , 4.9739932687524062e-05 , 5.5692601758028553e-05 , 6.2140388562404174e-05 , 6.9128401103763559e-05 , 7.6645878781258207e-05 , 8.4749185850655908e-05 , 9.3420483263056495e-05 , 0.00010273336557745961 , 0.00011265121488255643 , 0.0001232480108380004 , 0.00013459636946903071 , 0.00014650139507745382 , 0.00015929463718936134 , 0.00017284015891234515 , 0.00018715745983810908 , 0.00020226733820720105 , 0.0002182498128765064 , 0.00023512578116460171 , 0.00025292651954488888 , 0.00027165870771194241 , 0.00029134995579086707 , 0.00031202217045904067 , 0.00033370999234300623 , 0.00035644549456481851 , 0.00038025899861406803 , 0.00040518081307119023 , 0.0004312340532734016 , 0.0004584503987046613 , 0.00048684430822789812 , 0.00051645354569522907 , 0.0005472828402515276 , 0.00057938190608542079 , 0.00061275090975397421 , 0.0006474127548303588 , 0.00068343468905098371 , 0.00072076580050502625 , 0.00075947631501319265 , 0.00079958110240744553 , 0.0008410975840678226 , 0.00088402254995322081 , 0.00092838867936580823 , 0.00097421127156387751 , 0.0010215107156041454 , 0.0010702884133779744 , 0.0011205575804490652 , 0.0011723248197248871 , 0.0012256044570541449 , 0.0012804046232005564 , 0.0013367332506084143 , 0.0013945919856421184 , 0.0014539800651395143 , 0.001514901520821024 , 0.0015773496022817167 , 0.0016413334084849302 , 0.0017068354011726512 , 0.0017738657468903503 , 0.0018424107865905371 , 0.0019124473527039961 , 0.0019839905437757511 , 0.0020570044913864856 , 0.0021314772554310206 , 0.0022073935243709774 , 0.0022847435564257133 , 0.0023634945231977844 , 0.0024436264036744332 , 0.0025251133220893333 , 0.0026079357701856605 , 0.0026920587401893263 , 0.0027774521193260461 , 0.0028640791633547311 , 0.0029519081765462122 , 0.0030409006777833555 , 0.0031310205981640729 , 0.0032222250469656809 , 0.0033144685281284786 , 0.0034077089852878432 , 0.0035018951402814484 , 0.003596988201224169 , 0.0036929278229154137 , 0.0037896678870492553 , 0.0038871566241322287 , 0.0039853261730970861 , 0.0040841294169390663 , 0.0041835059838843079 , 0.0042833933387723512 , 0.0043837261368993262 , 0.0044844488974304479 , 0.0045854932957565195 , 0.0046867921209211068 , 0.0047882735931617116 , 0.0048898745755680383 , 0.0049915231788431611 , 0.0050931497285813497 , 0.0051946791297664465 , 0.0052960417422814373 , 0.0053971612955007083 , 0.0054979658306310002 , 0.0055983797994516738 , 0.0056983253408818286 , 0.0057977277900710658 , 0.0058965061241457476 , 0.0059945906110298215 , 0.0060918990398812472 , 0.0061883544421090396 , 0.0062838835100025494 , 0.0063784040723008813 , 0.0064718392683248841 , 0.0065641156880257012 , 0.0066551575218083692 , 0.0067448837961064734 , 0.0068332211341646538 , 0.0069200954455324835 , 0.0070054334691378312 , 0.0070891574289693657 , 0.0071711976967911263 , 0.0072514833710739506 , 0.0073299462238696437 , 0.0074065136345697734 , 0.0074811207162423544 , 0.0075536986382993528 , 0.00762418337802596 , 0.0076925122443991002 , 0.0077586234882858086 , 0.0078224582372105698 , 0.0078839551310774785 , 0.0079430619696299314 , 0.0079997234806384564 , 0.0080538863175640511 , 0.0081055017546585971 , 0.0081545228891952252 , 0.0082009008815607833 , 0.008244594453582119 , 0.0082855652960434623 , 0.0083237733919798405 , 0.0083591810162194566 , 0.0083917558821141533 , 0.0084214702626896412 , 0.0084482939561720712 , 0.0084722007770392724 , 0.0084931682078880754 , 0.0085111792876931842 , 0.0085262133908074918 , 0.0085382578791410729 , 0.0085473009945351702 , 0.0085533341831194563 , 0.0085563512135527313 , 0.0085563512135527313 , 0.0085533341831194563 , 0.0085473009945351702 , 0.0085382578791410729 , 0.0085262133908074918 , 0.0085111792876931842 , 0.0084931682078880754 , 0.0084722007770392724 , 0.0084482939561720712 , 0.0084214702626896412 , 0.0083917558821141533 , 0.0083591810162194566 , 0.0083237733919798405 , 0.0082855652960434623 , 0.008244594453582119 , 0.0082009008815607833 , 0.0081545228891952252 , 0.0081055017546585971 , 0.0080538863175640511 , 0.0079997234806384564 , 0.0079430619696299314 , 0.0078839551310774785 , 0.0078224582372105698 , 0.0077586234882858086 , 0.0076925122443991002 , 0.00762418337802596 , 0.0075536986382993528 , 0.0074811207162423544 , 0.0074065136345697734 , 0.0073299462238696437 , 0.0072514833710739506 , 0.0071711976967911263 , 0.0070891574289693657 , 0.0070054334691378312 , 0.0069200954455324835 , 0.0068332211341646538 , 0.0067448837961064734 , 0.0066551575218083692 , 0.0065641156880257012 , 0.0064718392683248841 , 0.0063784040723008813 , 0.0062838835100025494 , 0.0061883544421090396 , 0.0060918990398812472 , 0.0059945906110298215 , 0.0058965061241457476 , 0.0057977277900710658 , 0.0056983253408818286 , 0.0055983797994516738 , 0.0054979658306310002 , 0.0053971612955007083 , 0.0052960417422814373 , 0.0051946791297664465 , 0.0050931497285813497 , 0.0049915231788431611 , 0.0048898745755680383 , 0.0047882735931617116 , 0.0046867921209211068 , 0.0045854932957565195 , 0.0044844488974304479 , 0.0043837261368993262 , 0.0042833933387723512 , 0.0041835059838843079 , 0.0040841294169390663 , 0.0039853261730970861 , 0.0038871566241322287 , 0.0037896678870492553 , 0.0036929278229154137 , 0.003596988201224169 , 0.0035018951402814484 , 0.0034077089852878432 , 0.0033144685281284786 , 0.0032222250469656809 , 0.0031310205981640729 , 0.0030409006777833555 , 0.0029519081765462122 , 0.0028640791633547311 , 0.0027774521193260461 , 0.0026920587401893263 , 0.0026079357701856605 , 0.0025251133220893333 , 0.0024436264036744332 , 0.0023634945231977844 , 0.0022847435564257133 , 0.0022073935243709774 , 0.0021314772554310206 , 0.0020570044913864856 , 0.0019839905437757511 , 0.0019124473527039961 , 0.0018424107865905371 , 0.0017738657468903503 , 0.0017068354011726512 , 0.0016413334084849302 , 0.0015773496022817167 , 0.001514901520821024 , 0.0014539800651395143 , 0.0013945919856421184 , 0.0013367332506084143 , 0.0012804046232005564 , 0.0012256044570541449 , 0.0011723248197248871 , 0.0011205575804490652 , 0.0010702884133779744 , 0.0010215107156041454 , 0.00097421127156387751 , 0.00092838867936580823 , 0.00088402254995322081 , 0.0008410975840678226 , 0.00079958110240744553 , 0.00075947631501319265 , 0.00072076580050502625 , 0.00068343468905098371 , 0.0006474127548303588 , 0.00061275090975397421 , 0.00057938190608542079 , 0.0005472828402515276 , 0.00051645354569522907 , 0.00048684430822789812 , 0.0004584503987046613 , 0.0004312340532734016 , 0.00040518081307119023 , 0.00038025899861406803 , 0.00035644549456481851 , 0.00033370999234300623 , 0.00031202217045904067 , 0.00029134995579086707 , 0.00027165870771194241 , 0.00025292651954488888 , 0.00023512578116460171 , 0.0002182498128765064 , 0.00020226733820720105 , 0.00018715745983810908 , 0.00017284015891234515 , 0.00015929463718936134 , 0.00014650139507745382 , 0.00013459636946903071 , 0.0001232480108380004 , 0.00011265121488255643 , 0.00010273336557745961 , 9.3420483263056495e-05 , 8.4749185850655908e-05 , 7.6645878781258207e-05 , 6.9128401103763559e-05 , 6.2140388562404174e-05 , 5.5692601758028553e-05 , 4.9739932687524062e-05 , 4.4291245517772101e-05 , 3.9309720691864923e-05 , 3.4808770994987535e-05 , 3.076222082586599e-05 , 2.7190109935793652e-05 , 2.40808761912665e-05 , 2.1470175773501382e-05 , 1.936753284143648e-05 , 1.7833760715256158e-05 , 1.6914664698078393e-05 , 1.6714489931227413e-05 , 1.7335114024242748e-05 , -7.4029787723175851e-05]
        
            y = np.multiply(b, self.gauge_data)
            return y.sum()
        else:
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    config_hardware_server = ConfigHardwareService()
    rclpy.spin(config_hardware_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
