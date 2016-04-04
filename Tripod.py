# -*- coding: utf-8 -*-

from twisted.internet import reactor, task
from Config import Config
from Canopen import Canopen
from Kinematic import Kinematic
import os
import re
from time import sleep
import stat
import logging
import ConfigParser

import pygame

class Tripod():
    """Classe principale per la gestione del tripode.

    Per la gestione del tripode, sono necessarie tre distinte attivita':

      - Abilitazione dei segnali visivi e acustici in base allo stato
      - Esecuzione del programma canopen_server per la comunicazione con i motori
      - Avvio del server TCP per la comunicazione con il programma di controllo remoto

    Il controllo dei segnali visivi ed acustici avviene tramite bus i2c, mentre il loro
    stato dipende da quanto comunicato tramite PIPE dal canopen_server.

    Il dialogo con il programma di controllo dei motori avviene tramite stdin, stdout e PIPE.

    """

    def __init__(self):

        logging.info("Initializing ALMA_Tripod class")

        self.ini = ConfigParser.ConfigParser()
        self.ini.read('/opt/spinitalia/service/config.ini')

        # Istanzio le classi
        self.config = Config()
        self.canopen = Canopen(self)
        self.kinematic = Kinematic(self)

        self.almaPositionProtocol = None

        self.last_sim = ""
        self.last_sim_file = None
        self.last_sim_time = 0.0

        # All'avvio il sistema si trova nello stato 3, ATTIVO
        self.canStatus = '3'
        self.oldStatus = 'NOS'    # Forzo il primo update
        self.simHash = ''

        self.userLoggedIn = False
        self.isAsyncError = False

        # Inizializza le strutture
        self.motorsAddress = []
        self.motorPos = {'120': 0L, '121': 0L, '122': 0L, '123': 0L, '119': 0L}
        self.motorPosLimit = {'120': 440000, '121': 440000, '122': 140000, '123': 140000, '119': 440000}
        self.posTime = 0.0
        self.OpProgress = 0
        self.isCentered = False
        self.isImporting = False
        self.currentLine = 0
        self.mex_counter = 0L
        self.almaControlProtocol = None
        self.isReading = False

        self.antenna_weight = 50

        # Indirizzi dei motori
        self.motor_address_list = ['119', '120', '121', '122', '123']

        # Calcolo della velocità
        self.old_roll = 0.0
        self.old_pitch = 0.0
        self.old_yaw = 0.0

        # Compila i cercatori
        self.find_motor_position = re.compile(
            '@M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) AS([^ ]*) T([^ ]*) C([^ ]*)'
        )

        self.joy_call = None

        # Per prima cosa aggiorno lo stato
        self.update_output()

        self.motor_file = dict()
        self.joy_value = dict()
        self.joy_direction = 1
        self.joy_device = None

        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()

        logging.info("Numer of joysticks: {}".format(joystick_count))
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            logging.info("Joystick {}".format(i))

            name = joystick.get_name()
            logging.info("Joystick name: {}".format(name))

            if self.joy_device is None:
                if name[:4] == "Xbox":
                    self.joy_device = pygame.joystick.Joystick(i)

    def cleanup(self):

        logging.info("De-Initializing ALMA_Tripod class")

        # Invia CT6, spegnimento
        reactor.callFromThread(self.canopen.sendCommand, 'CT6')

        # Chiude lo streamer
        if 'almaPositionProtocol' in vars():
            if self.almaPositionProtocol.posSender:
                self.almaPositionProtocol.posSender.stop()

        # Chiude lo stream_reader
        if 'receiver_thread' in vars():
            self.receiver_thread.stop()

        # Chiude il processo di comunicazione con i motori se attivo
        if self.canopen:
            self.canopen.transport.closeStdin()

        # Chiude il reattore se attivo
        if reactor.running:
            logging.info("Stopping reactor")
            reactor.stop()

    def start_canopen(self):

        # Avvio il server CANOPEN
        # usePTY serve ad evitare l'ECHO
        # INFO: uso stdbuf per evitare il buffering dell'output se non in terminale
        if self.config.isFake:
            reactor.spawnProcess(
                self.canopen,
                "/usr/bin/stdbuf",
                args=[
                    "stdbuf",
                    "--output=L",
                    "--input=0",
                    "{}alma3d_canopenshell".format(self.config.INSTALL_PATH),
                    "fake",
                    "load#libcanfestival_can_socket.so,0,1M,8"
                ],
                env=os.environ,
                usePTY=False
            )
        else:
            reactor.spawnProcess(
                self.canopen,
                "/usr/bin/stdbuf",
                args=[
                    "stdbuf",
                    "--output=L",
                    "--input=0",
                    "{}alma3d_canopenshell".format(self.config.INSTALL_PATH),
                    "load#libcanfestival_can_socket.so,0,1M,8"
                ],
                env=os.environ,
                usePTY=False
            )

    def update_import_progress(self, value, line_num):

        self.OpProgress = value
        self.currentLine = line_num

    def update_import_end(self, md5sum):

        self.isImporting = False
        self.last_sim = md5sum

    def stream_reader(self):

        # TODO: Deve ripartire automaticamente in caso di errore ed in caso di assenza di pipe!
        logging.info("Position reader thread started!")

        isPipeOpen = False

        motorPos = {'120': 0L, '121': 0L, '122': 0L, '119': 0L}
        isAsyncError = False
        canStatus = '3'
        isCentered = False
        OpProgress = 0
        mex_counter = 0L
        posTime = 0.0

        while self.isReading:
            if isPipeOpen:
                try:
                    line = pipein.readline()[:-1]
                    # print 'Parent %d got "%s" at %s' % (os.getpid(), line, time.time( ))
                    # line: @M119 S0 @M120 S0 @M121 S0 @M122 S0 AS4 T9 C0
                    canopen_status = self.find_motor_position.search(line)
                    if canopen_status:
                        motorPos[canopen_status.group(1)] = canopen_status.group(2)
                        motorPos[canopen_status.group(3)] = canopen_status.group(4)
                        motorPos[canopen_status.group(5)] = canopen_status.group(6)
                        motorPos[canopen_status.group(7)] = canopen_status.group(8)
                        motorPos[canopen_status.group(9)] = canopen_status.group(10)

                        # Se lo stato e' zero, vuol dire che c'e' una segnalazione pendente
                        if canopen_status.group(11) == '0' and isAsyncError is False:
                            isAsyncError = True
                        elif canopen_status.group(11) != '0':
                            canStatus = canopen_status.group(11)
                            if not isCentered:
                                if canStatus == '6':
                                    isCentered = True
                        posTime = float(canopen_status.group(12))
                        OpProgress = canopen_status.group(13)
                        mex_counter = mex_counter + 1

                    reactor.callFromThread(
                        self.update_var_from_canopen, motorPos, isAsyncError, canStatus, isCentered,
                        posTime, OpProgress, mex_counter
                    )

                except Exception, e:
                    isFileOpen = False
                    logging.info("Pipe closed!")

            else:
                try:
                    mode = os.stat(self.config.POS_PIPE).st_mode
                    if stat.S_ISFIFO(mode):
                        pipein = open(self.config.POS_PIPE, 'r')
                        isPipeOpen = True
                        continue
                    sleep(0.5)

                except:

                    pass

    def update_var_from_canopen(self, motorPos, isAsyncError, canStatus, isCentered, posTime, OpProgress, mex_counter):

        self.motorPos = motorPos
        self.isAsyncError = isAsyncError
        self.canStatus = canStatus
        self.isCentered = isCentered
        self.posTime = posTime
        if not self.isImporting:
            self.OpProgress = OpProgress
        self.update_output()

        if self.almaPositionProtocol is not None:
            self.mex_counter = mex_counter
            self.almaPositionProtocol.send_position()

    def goto_em2(self):

        reactor.callFromThread(self.canopen.sendCommand, 'EM2', "local")

    def send_tcp_response(self, data):

        if((data[:6] == "OK CT4") and (self.joy_call is not None)):
            logging.info('Virtual joystick stopped')
            self.joy_call.stop()

        try:
            self.almaControlProtocol.sendResponse(data)

            # Se e' stato ricevuto l'ok sullo spegnimento, spengo il reattore
            # if data[:7] == "OK CT6":

        except:
            pass

    @staticmethod
    def translate_status(status):

        if status == '0':
            return 'ERRORE'
        elif status == '1':
            return 'SPENTO'
        elif status == '2':
            return 'EMERGENZA'
        elif status == '3':
            return 'ATTIVO'
        elif status == '4':
            return 'INIZIALIZZATO'
        elif status == '5':
            return 'RICERCA_CENTRO'
        elif status == '6':
            return 'CENTRATO'
        elif status == '7':
            return 'ANALIZZATO'
        elif status == '8':
            return 'SIMULAZIONE'
        elif status == '9':
            return 'FERMO'
        elif status == '10':
            return 'CENTRAGGIO'
        elif status == '11':
            return 'RILASCIATO'
        elif status == '12':
            return 'NON_AUTENTICATO'
        elif status == '13':
            return 'IN_POSIZIONE'
        elif status == '14':
            return 'JOYSTICK_COLLEGATO'
        elif status == '15':
            return 'MOVIMENTO_LIBERO'

    def virtual_joystick(self):

        if self.canStatus == '14':

            # controllo che sia presente il joystick
            if self.joy_device is None:
                pygame.joystick.init()
                joystick_count = pygame.joystick.get_count()

                logging.info("Numer of joysticks: {}".format(joystick_count))
                for i in range(joystick_count):
                    joystick = pygame.joystick.Joystick(i)
                    joystick.init()
                    logging.info("Joystick {}".format(i))

                    name = joystick.get_name()
                    logging.info("Joystick name: {}".format(name))

                    if name[:4] == "Xbox":
                        self.joy_device = pygame.joystick.Joystick(i)

            for motor in self.motor_address_list:
                file_name = '{}{}{}'.format(self.config.MOT_DATA, motor, self.config.MOT_EXT)

                if motor not in self.motor_file:
                    logging.info("Pipe {} opened".format(file_name))
                    self.motor_file[motor] = open(file_name, "w", 0)

        elif self.canStatus == '15':
            if self.joy_device is not None:

                # EVENT PROCESSING STEP
                for event in pygame.event.get():  # User did something
                    if event.type == pygame.QUIT:  # If user clicked close
                        done = True  # Flag that we are done so we exit this loop
                    # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
                    if event.type == pygame.JOYBUTTONDOWN:
                        print("Joystick button pressed.")
                    if event.type == pygame.JOYBUTTONUP:
                        print("Joystick button released.")

            else:
                if abs(self.joy_value) == self.motorPosLimit['119']:
                    self.joy_direction *= -1

                if self.joy_direction == 1:
                    self.joy_value += 4000
                else:
                    self.joy_value -= 4000

            # le linee da mettere nella pipe voglio che siano limitate, sicuramente minori
            # della massima lunghezza della tabella dell'interpolatore (per motivi di delay)
            # In questo caso, vorrei al massimo 25 linee, scelte tra un tradoff tra la latenza
            # di controllo e la velocità del loop di trasmissione in modo da non ottenere un errore
            # di underflow
            line_to_send = int(25 - float(self.OpProgress))

            if line_to_send > 0:
                for motor in self.motor_address_list:
                    if self.joy_device is not None:
                        if motor == '119':
                            axis = self.joy_device.get_axis(0)
                        elif motor == '120':
                            axis = self.joy_device.get_axis(1)
                        else:
                            axis = 0

                        if motor not in self.joy_value:
                            self.joy_value[motor] = 0

                        self.joy_value[motor] += axis * 4000

                        if self.joy_value[motor] > self.motorPosLimit[motor]:
                            self.joy_value[motor] = self.motorPosLimit[motor]
                        elif self.joy_value[motor] < -self.motorPosLimit[motor]:
                            self.joy_value[motor] = -self.motorPosLimit[motor]

                    try:
                        for i in range(0,line_to_send):
                            self.motor_file[motor].write("CT1 M{} S{} T10\n".format(motor, self.joy_value[motor]))

                    except IOError as ex:
                        logging.info('Virtual joystick stopped')
                        self.joy_call.stop()

                        print ex

                        for motor in self.motor_address_list:
                            if motor in self.motor_file:
                                self.motor_file[motor].close()
                                del self.motor_file[motor]

                                file_name = '{}{}{}'.format(self.config.MOT_DATA, motor, self.config.MOT_EXT)
                                logging.info("Pipe {} removed".format(file_name))

                                if os.path.exists(file_name):
                                    os.remove(file_name)

                        break



    def update_output(self):
        """Aggiorna le uscite in base al nuovo stato
        Sono presenti due stati, quello del server e quello della classe. Normalmente coincidono,
        ma in alcune condizioni divergono ( can='ERRORE', class=stato_in_cui_si_verifica ).
        Questa funzione viene chiamata se e solo se lo stato del sever CANOpen e' cambiato."""

        # Gli stati avanzano in baso allo stream del canopen_server, ad eccezione di NON_AUTENTICATO
        # e ANALIZZATO

        # In base al nuovo stato eseguo delle operazioni
        # Verde: Acceso
        # Giallo: Movimento reale o potenziale
        # Rosso: Errore
        # Beep: Movimento reale
        # Green: Attivo
        #                    Verde  Giallo  Rosso  Buzzer  Recinto  Torque
        # 0 - Errore           I       I      I       I       I       I     I = Invariato
        # 1 - SPENTO           -       -      -       -       -       G     G = Gravita'
        # 2 - EMERGENZA        X       -      X       -       -       T     T = Tenuta
        # 3 - ATTIVO           X       X      -       -       -       T     X = On
        # 4 - INIZIALIZZATO    X       -      -       -       X       T     0 = Libero
        # 5 - RICERCA_CENTRO   X       X      -       X       X       T
        # 6 - CENTRATO         X       X      -       -       X       T
        # 7 - ANALIZZATO       X       X      -       -       X       T
        # 8 - SIMULAZIONE      X       X      -       X       X       T
        # 9 - FERMO            X       X      -       -       X       T
        # A - CENTRAGGIO       X       X      -       X       X       T
        # B - RILASCIATO       X       -      X       X       -       0     * Il recinto se da TCP compare EM1
        # C - NON_AUTENTICATO  X       X      -       -       X       T

        # Thread safe!
        if self.oldStatus != self.canStatus:

            # Stampo l'evento
            if len(self.motorPos) > 0:
                motor_yaw = self.motorPos['119']
                motor_front = self.motorPos['120']
                motor_rear_right = self.motorPos['121']
                motor_rear_left = self.motorPos['122']
            else:
                motor_yaw = 0
                motor_front = 0
                motor_rear_right = 0
                motor_rear_left = 0

            logging.info("Position: (%s, %s, %s, %s), Status: %s (%s), Time: %s, Completed: %s" % (
                motor_yaw,
                motor_front,
                motor_rear_right,
                motor_rear_left,
                self.translate_status(self.canStatus),
                self.canStatus,
                self.posTime, self.OpProgress))

            self.oldStatus = self.canStatus
