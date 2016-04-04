# -*- coding: utf-8 -*-
from twisted.protocols.basic import LineReceiver
from twisted.internet import task
import logging
import re
import time
import os
import shutil
import Config
import subprocess


class ControlProtocol(LineReceiver):
    """Gestione del protocollo ALMA
    
    Questa classe gestisce la comunicazione con il client TCP.
    
    Si occupa di rispondere immediatamente a comandi particolari, e di dirottare
    gli altri al processo canopen_server.
    
    """
    
    delimiter = '\n'

    def __init__(self, tripod):
    
        logging.info("Initializing ALMA_Protocol class")

        # Carico il file di configurazione
        self.config = Config.Config()
        
        # Inizializzo le variabili
        self.old_status = 99 #################################
        self.can_status = '' #################################
        self.tripod = tripod
        self.name = None
        self.isBusy = False
        self.last_command = ''
        self.last_login = 0
        self.login_pause = 1
        
        # Se non configurato entra direttamente senza richiedere il login
        if not self.config.isLoginRequired:
            self.tripod.userLoggedIn = True
            self.user = 'alma_user'
            logging.info("Pre-logged in with alma_user")
        else:
            self.tripod.userLoggedIn = False
            self.user = ''

        # Compila i cercatori
        self.find_weight = re.compile('CT0 W(\d+)')
        self.find_motor_position = re.compile('@M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) '
                                              'S([^ ]*) AS([^ ]*) T([^ ]*) C([^ ]*)')
        self.find_motor_address = re.compile('@M A([\d]*)')
        self.find_end_position = re.compile('CT1 R([+-]?(?:\d+\.\d*|\.\d+|\d+)) P([+-]?(?:\d+\.\d*|\.\d+|\d+)) '
                                            'Y([+-]?(?:\d+\.\d*|\.\d+|\d+)) V(\d+)')
        self.find_ip = re.compile('PR4 ([^ ]*) ([^ ]*) ([^ ]*)')

    def connectionMade(self):
    
        # Controllo che non sia gia' connesso qualcuno
        if self.tripod.isClientConnected:
            logging.info("Connection request but already connected from %s" % self.tripod.connectedAddress)
            self.sendLine("#Ip address %s already connected" % self.tripod.connectedAddress)
            self.transport.loseConnection()
            return
        else:
            logging.info("Connection request from %s" % self.tripod.connectedAddress)
            self.tripod.isClientConnected = True

    def connectionLost(self, reason):

        print "Connection lost with TCP client ({})!".format(reason)
        
        self.tripod.isClientConnected = False
        self.tripod.connectedAddress = ""

    def lineReceived(self, line):
        """Ricezione dei comandi dal client TCP

        Qui devo servire le richieste del cliente, che possono essere di login o comandi da
        rigirare al servizio CANOpen.

        In alcuni casi particolari devo eseguire azioni locali, come quando viene re-inizializzato
        il sistema a seguito di un allarme, e devo spegnere l'allarme stesso.

        Inoltre per il comando CT1 R32.100 P12.000 Y305, devo

        :type self: object
        """
        
        logging.info("From TCP Client '%s' (status '%s' is '%s')" % (
            line,
            self.tripod.canStatus,
            self.tripod.translate_status(self.tripod.canStatus))
        )

        # Se non si e' loggato qualcuno, non accetto alcun comando
        if not self.tripod.userLoggedIn:        

            login = line.split(" ")
            time_elapsed = time.time() - self.last_login
            if time_elapsed < self.login_pause:
                self.sendLine("CERR LGN 1: Last login attemp was wrong, waiting %s seconds" % self.login_pause)
            elif login[0] == "LGN" and login[1] == "alma_user" and login[2] == "spinitalia":
                self.user = login[1]
                self.login_pause = 1
                logging.info("User %s logged in" % login[1])
                self.sendLine("OK LGN")
                self.tripod.userLoggedIn = True
            elif login[0] == "LGN" and login[1] == "alma_admin" and login[2] == "spinitalia":
                self.user = login[1]
                self.login_pause = 1
                logging.info("User %s logged in" % login[1])
                self.sendLine("OK LGN")
                self.tripod.userLoggedIn = True
            else:
                self.sendLine("CERR LGN 0: Username and/or password wrong")
                self.last_login = time.time()
                self.login_pause = self.login_pause * 2
                logging.info("Wrong login attemp, waiting %ss" % self.login_pause)
                
        else:
            # Se viene richiesta l'inizializzazione catturo il peso dell'antenna
            if line.rstrip().upper()[:3] == 'CT0':

                # Prendo il peso
                antenna_weight = self.find_weight.search(line.rstrip())
                if antenna_weight:
                    self.tripod.antenna_weight = antenna_weight.group(1)
                
                # Mando il comando giusto
                line = "CT0 M5"

            # Se viene richiesto l'avvio della simulazione, converto in coordinate motore ed invio i comandi
            elif line.rstrip().upper()[:5] == 'CT1 R':

                matches = self.find_end_position.search(line.rstrip().upper())
                if matches:
                    #self.sendLine("Roll:{}, Pitch:{}, Yaw:{}".format(matches.group(1), matches.group(2),
                    #                                                 matches.group(3)))
                    if self.tripod.kinematic.convert_point(matches.group(1), matches.group(2), matches.group(3)):
                        # self.tripod.kinematic.last_conversion_steps
                        # Dovrei mandare
                        # VT_R = Velocity * 65536 * 360 / 115     ( Gradi al secondo   )
                        # AT_R = Acceleration * 8192 * 360 / 115  ( Gradi al secondo^2 )
                        # CT1 M119 P{} VM{} AM{}".format(self.tripod.kinematic.last_conversion_steps[3], VT_R, VT_R)
                        # CT1 M120 P1231232 VM123 AM124
                        # CT1 M121 P1231232 VM123 AM124
                        # CT1 M122 P1231232 VM123 AM124
                        self.tripod.canopen.is_sending_position = 1
                        self.tripod.canopen.relative_speed = 0.1
                        if matches.group(4).isNumeric():
                            if 0 < int(matches.group(4)) < 100:
                                self.tripod.canopen.relative_speed = matches.group(4)
                        VT_R = int(self.tripod.canopen.VT_R / 100.0 * self.tripod.canopen.relative_speed)
                        AT_R = int(self.tripod.canopen.AT_R / 100.0 * self.tripod.canopen.relative_speed)
                        line = "CT1 M119 P{} VM{} AM{}".format(self.tripod.kinematic.last_conversion_steps[3], VT_R,
                                                               AT_R)
                        self.tripod.canopen.sendCommand(line, "self")
                        return

            # Se viene richiesto l'homing carico i file necessari predefiniti
            elif line.rstrip().upper() == 'CT2 P1':

                if self.tripod.canStatus == '4':

                    logging.info("Copia dei comandi di homing")
                    if not os.path.exists(self.config.MOT_DATA):
                        os.makedirs(self.config.MOT_DATA)
                    # TODO: Controllare che inizi con @M

                    for motor_address in self.tripod.motorsAddress:
                        file_name = '{}{}{}'.format(self.tripod.config.MOT_DATA, motor_address, self.tripod.config.MOT_EXT)

                        if os.path.exists(file_name):
                            os.remove(file_name)

                        self.tripod.motor_file[motor_address] = open(file_name, "w+")
                        os.chmod(file_name, 0666)

                        self.tripod.motor_file[motor_address].write("CT1 M{} H-320000 VF300000 VB900000".format(motor_address))

                        self.tripod.motor_file[motor_address].close()
                        del self.tripod.motor_file[motor_address]

                    # Annullo la simulazione memorizzata
                    self.tripod.last_sim = ""

                else:

                    self.sendLine("CERR CT2 P1: Stato del sistema non valido")
                    return

            # Se viene richiesto l'homing carico i file necessari predefiniti
            elif line.rstrip().upper() == 'CT2 P4':

                self.tripod.canopen.is_lowering = 1
                line = "CT1 M119 P{} VM{} AM{}".format(
                    0,
                    int(self.tripod.canopen.VT_R / 10),
                    int(self.tripod.canopen.AT_R / 10)
                )
                self.tripod.canopen.sendCommand(line, "self")

            # Se viene richiesta l'analisi di un file, lo analizzo
            elif line.rstrip().upper()[:3] == 'CT3':

                if all([self.tripod.canStatus == '6', self.tripod.isImporting is False]):
                    self.tripod.kinematic.start_parsing(line.rstrip(), self)
                    self.last_command = line.rstrip().upper()
                    return
                else:
                    self.sendLine("ERR CT3 0: Already busy")
                    return

            # Se viene richiesta l'apertura del joystick
            elif line.rstrip().upper()[:3] == 'CB3':
                if (self.tripod.canStatus == '4') or (self.tripod.canStatus == '6') or (self.tripod.canStatus == '9'):
                    logging.info("Open joystick pipes")
                    # Se non esiste la directory la crea
                    if not os.path.exists(self.tripod.config.MOT_DATA):
                        os.umask(0)
                        os.makedirs(self.tripod.config.MOT_DATA, 0777)
                    if not os.path.exists(self.tripod.config.LOG_PATH):
                        os.umask(0)
                        os.makedirs(self.tripod.config.LOG_PATH, 0777)
                    if not os.path.exists(self.tripod.config.SIM_PATH):
                        os.umask(0)
                        os.makedirs(self.tripod.config.SIM_PATH, 0777)

                    for motor in self.tripod.motor_address_list:
                        file_name = '{}{}{}'.format(self.tripod.config.MOT_DATA, motor, self.tripod.config.MOT_EXT)
                        logging.info("Pipe {} created".format(file_name))

                        if os.path.exists(file_name):
                            logging.info('File {} removed before create fifo'.format(file_name))
                            os.remove(file_name)

                        os.mkfifo(file_name)

                    logging.info('Virtual joystick started')
                    self.tripod.joy_call = task.LoopingCall(self.tripod.virtual_joystick)
                    self.tripod.joy_call.start(0.1)


            # Se si avvia la simulazione, inizio a scrivere il log
            elif line.rstrip().upper()[:3] == 'CT4':

                md5sum = self.tripod.last_sim
                filename = "{}posizione_motori_{}.csv".format(self.tripod.config.LOG_PATH, md5sum)
                self.tripod.last_sim_file = open(filename, "w+")
                os.chmod(filename, 0666)
                self.tripod.last_sim_file.write("Counter;Time;Roll_DK;Pitch_DK;Yaw_DK;Step_119_Motor;Step_120_Motor;"
                                                "Step_121_Motor;Step_122_Motor\n")
                self.tripod.last_sim_time = 0.0
                self.tripod.mex_counter = 0

            elif line.rstrip().upper()[:3] == 'CT5' or line.rstrip().upper()[:3] == 'CB5':
                if self.tripod.joy_call is not None:
                    logging.info('Virtual joystick stopped')
                    self.tripod.joy_call.stop()

                for motor in self.tripod.motor_address_list:
                    self.tripod.motor_file[motor].close()
                    del self.tripod.motor_file[motor]

                    file_name = '{}{}{}'.format(self.tripod.config.MOT_DATA, motor, self.tripod.config.MOT_EXT)
                    logging.info("Pipe {} removed".format(file_name))

                    if os.path.exists(file_name):
                        os.remove(file_name)


            # Se era CT6, spegne il protocollo
            elif line.rstrip().upper()[:3] == 'CT6':
                #self.transport.loseConnection()
                process = subprocess.Popen("/sbin/poweroff", stdout=subprocess.PIPE)
                output = process.communicate()[0]
                logging.info("Poweroff says: '{}'".format(output))
                return

            # Se era PR7, invio la simulazione memorizzata
            elif line.rstrip().upper()[:3] == 'PR7':

                if self.tripod.last_sim == "":
                    self.transport.write("CERR PR7 0: No simulation loaded\n")
                else:
                    self.transport.write("OK PR7 {}\n".format(self.tripod.last_sim))
                return

            # Se era PR7, invio la simulazione memorizzata
            elif line.rstrip().upper()[:3] == 'PR4':

                matches = self.find_ip.search(line.rstrip().upper())
                if matches:
                    network_file = open("/etc/network/interfaces", "w")
                    network_file.write("auto lo\n")
                    network_file.write("iface lo inet loopback\n")
                    network_file.write("\n")
                    network_file.write("# Versione con DHCP attivato\n")
                    network_file.write("#auto eth0\n")
                    network_file.write("#allow-hotplug eth0\n")
                    network_file.write("#iface eth0 inet dhcp\n")
                    network_file.write("\n")
                    network_file.write("# Versione con IP statico\n")
                    network_file.write("auto eth0\n")
                    network_file.write("iface eth0 inet static\n")
                    network_file.write("  address {}\n".format(matches.group(1)))
                    network_file.write("  netmask {}\n".format(matches.group(2)))
                    network_file.write("  gateway {}\n".format(matches.group(3)))
                    network_file.write("\n")
                    network_file.write("#auto wlan0\n")
                    network_file.write("#allow-hotplug wlan0\n")
                    network_file.write("#iface wlan0 inet manual\n")
                    network_file.write("#wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf\n")
                    network_file.write("\n")
                    network_file.write("#auto wlan1\n")
                    network_file.write("#allow-hotplug wlan1\n")
                    network_file.write("#iface wlan1 inet manual\n")
                    network_file.write("#wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf\n")
                    network_file.write("\n")
                    network_file.write("auto can0\n")
                    network_file.write("iface can0 inet manual\n")
                    network_file.write("pre-up /sbin/ip link set can0 qlen 1000 type can bitrate 1000000 triple-sampling on\n")
                    network_file.write("up /sbin/ifconfig can0 up\n")
                    network_file.write("down /sbin/ifconfig can0 down\n")
                    network_file.close()

                    # matches.group(1) -> IP
                    # matches.group(2) -> Subnet
                    # matches.group(3) -> Getway
                    self.transport.write("OK PR4 (IP:{}, SUBNET:{}, GETWAY:{})\n".format(
                        matches.group(1),
                        matches.group(2),
                        matches.group(3)
                    ))
                    subprocess.check_output(["/etc/init.d/networking", "reload"])
                    return

            self.tripod.canopen.sendCommand(line, "remote")
            self.last_command = line.rstrip().upper()

    def is_number(self, s):
        try:
            float(s)
            return True
        except ValueError:
            pass

        try:
            import unicodedata
            unicodedata.numeric(s)
            return True
        except (TypeError, ValueError):
            pass

        return False
            
    def sendResponse(self, response):
        """Invio delle risposte al client TCP
        Vengono qui inviate le risposte al client TCP provenienti generalmente dal servizio CANOpen
        o dalla classe stessa qualora si sia reso necessario gestire allarmi o quant'altro sia
        indipendente dai motori.
        """

        # Elimino \n dalla risposta inviata dal server CANOpen
        response_list = response.rstrip().split("\n")
        logging.info("To TCP client '%s' (command was '%s')" % (response_list, self.last_command))

        # Nel caso dell'inizializzazione catturo gli indirizzi dei motori trovati
        if self.last_command == "CT0 M4" and response_list[0][:2] == "@M":
            # TODO: Controllare che inizi con @M
            try:
                matches = self.find_motor_address.findall(response)
                for motor_address in matches:
                    self.tripod.motorsAddress.append(motor_address)
                    self.tripod.motorPos[motor_address] = 0L
                    logging.info("Found motor with address %s" % motor_address)
            except:
                logging.error("Errore nel parsing degli indirizzi")
        else:
            # Negli altri casi rigiro la risposta al comando
            self.sendLine(response.rstrip())

        # print "self.last_command: '%s', response_list[0][:2]: '%s'" %(self.last_command, response_list[0][:2])
