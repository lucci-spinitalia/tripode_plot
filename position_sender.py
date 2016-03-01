#!/usr/bin/env python
# -*- coding: utf-8 -*-

from twisted.internet import protocol, reactor, task
from twisted.protocols.basic import LineReceiver
from time import sleep

import re, os, signal, threading, logging, stat

class ALMA_Config():

    def __init__(self):

        self.isFake = True
        self.isLoginRequired = False
        self.FENCE_PIN = 12
        self.INSTALL_PATH = "/opt/spinitalia/"
        self.SIM_PATH = "/home/spinitalia/simulazioni/"
        self.DEF_MOVE = "/opt/spinitalia/default_position/"
        self.MOT_DATA = "/tmp/spinitalia/motor_data/"
        self.MD5SUM_EXEC = '/usr/bin/md5sum'
        if self.isFake:
            self.MOT_EXT = ".mot.fake"
        else:
            self.MOT_EXT = ".mot"
        if self.isFake:
            self.POS_PIPE = "/tmp/fake_alma_3d_spinitalia_pos_stream_pipe"
        else:
            self.POS_PIPE = "/tmp/alma_3d_spinitalia_pos_stream_pipe"



class ALMA_Canopen(protocol.ProcessProtocol):
    """Gestione del server CANOPEN
    Implementa il wrapper per il controllo della comunicazione CANOPEN, che viene qui creato un un processo
    esterno, catturando opportunamente STDIN e STDOUT.
    """

    def __init__(self, tripod):

        logger.info("Initializing ALMA_Canopen class")

        self.data = ""
        self.last_command = ""
        self.tripod = tripod

        self.connectionMade()

    def sendCommand(self, command):

        if self.connected:
            self.transport.write("%s\n" % command)
            self.last_command = command
            logger.info("To Canopen '%s\\n'" % command)

    def connectionMade(self):

        logger.info("ConnectionMade with CANOpen!")
        
        # Avvio il thread per la lettura delle posizioni
        self.tripod.isReading = True
        self.tripod.receiver_thread = threading.Thread(target=self.tripod.stream_reader)
        self.tripod.receiver_thread.setDaemon(True)
        self.tripod.receiver_thread.start()
        
    def outReceived(self, data):
        """Sono stati ricevuti bytes su STDOUT dal sotto-processo
        Il sotto-processo ha appena inviato su stdout delle informazioni, che rigiro a chi connesso via TCP"""
        
        self.tripod.send_tcp_response(data)        
        logger.info("From Canopen '%s'" % data.replace("\n","\\n"))

    def errReceived(self, data):
        """Sono stati ricevuti bytes su STDERR dal sotto-processo
        Il sotto-processo ha appena inviato su stderr delle informazioni"""

        logger.info("errReceived! with %d bytes! %s" % (len(data), data))

    def inConnectionLost(self):

        logger.info("inConnectionLost! stdin is closed! (we probably did it)")

    def outConnectionLost(self):

        logger.info("outConnectionLost! The child closed their stdout!")
        
    def errConnectionLost(self):

        logger.info("errConnectionLost! The child closed their stderr.")

    def processExited(self, reason):

        if reason.value.exitCode:
            logger.info("processExited, exit code %d" % (reason.value.exitCode,))
        else:
            logger.info("processExited")

    def processEnded(self, reason):

        logger.info("processEnded, exit code %d" % (reason.value.exitCode,))
        if reactor.running:
            reactor.stop()


class ALMA_Stream_Protocol(protocol.Protocol):
    """Protocollo per l'invio delle posizioni ALMA"""

    def __init__(self, tripod):
        self.tripod = tripod

    def connectionMade(self):
        self.tripod.stream_factory.numProtocols = self.tripod.stream_factory.numProtocols + 1 
        self.posSender = task.LoopingCall(self.sendPosition)
        self.posSender.start(0.01)
        logger.info("Welcome! There are currently %d open connections" % (self.tripod.stream_factory.numProtocols))

    def connectionLost(self, reason):
        self.tripod.stream_factory.numProtocols = self.tripod.stream_factory.numProtocols - 1

    def sendPosition(self):

        line = self.tripod.line

        self.transport.write("%s\n" % (line))

class ALMA_Streamer_Factory(protocol.Factory):
    """Factory di gestione del protocollo di invio delle posizioni ALMA"""

    def __init__(self, tripod):
        # Devo impedire il collegamento multiplo!
        self.tripod = tripod
        self.tripod.stream_factory = self
        self.numProtocols = 0
    
    def buildProtocol(self, addr):
        self.tripod.almaStreamProtocol = ALMA_Stream_Protocol(self.tripod)
        return self.tripod.almaStreamProtocol

class ALMA_Tripod():
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
    
        logger.info("Initializing ALMA_Tripod class")

        # Istanzio le classi
        self.config = ALMA_Config()
        self.canopen = ALMA_Canopen(self)
        
        # All'avvio il sistema si trova nello stato 3, ATTIVO
        self.canStatus = '3'
        self.oldStatus = 'NOS'    # Forzo il primo update
        self.simHash = ''
        
        self.userLoggedIn = False
        self.isAsyncError = False
        
        # Inizializza le strutture
        self.line = ''
        self.motorPos = {}
        self.posTime = 0
        self.posProgress = 0
        
        # Compila i cercatori
        self.find_motor_position = re.compile('@M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) AS([^ ]*) T([^ ]*) C([^ ]*)')
        
        # Per prima cosa aggiorno lo stato
        self.update_output()
        
    def cleanup(self):
    
        logger.info("De-Initializing ALMA_Tripod class")

        # Chiude lo streamer
        if 'almaStreamProtocol' in vars():
            if self.almaStreamProtocol.posSender:
                self.almaStreamProtocol.posSender.stop()
                
        # Chiude lo stream_reader
        if 'receiver_thread' in vars():
            self.receiver_thread.stop()

        # Chiude il processo di comunicazione con i motori se attivo
        #if self.canopen:
        #    self.canopen.transport.closeStdin()
            
        # Chiude il reattore se attivo
        if reactor.running:
            logger.info("Stopping reactor")
            reactor.stop()


    def start_canopen(self):

        # Avvio il server CANOPEN
        # usePTY serve ad evitare l'ECHO
        # INFO: uso stdbuf per evitare il buffering dell'output se non in terminale
        if self.config.isFake:
            reactor.spawnProcess(self.canopen, "/usr/bin/stdbuf", args=["stdbuf", "--output=L", "--input=0",
                "/opt/spinitalia/canopenshell", "fake",
                "load#libcanfestival_can_socket.so,0,1M,8"], env=os.environ, usePTY=False)
        else:
            reactor.spawnProcess(self.canopen, "/usr/bin/stdbuf", args=["stdbuf", "--output=L", "--input=0",
                "/opt/spinitalia/canopenshell",
                "load#libcanfestival_can_socket.so,0,1M,8"], env=os.environ, usePTY=False)

    def stream_reader(self):

        # TODO: Deve ripartire automaticamente in caso di errore ed in caso di assenza di pipe!
        logger.info("Position reader thread started!")
        
        isPipeOpen = False
        
        while self.isReading:
            if isPipeOpen:
                self.line = pipein.readline()[:-1]

                #print 'Parent %d got "%s" at %s' % (os.getpid(), line, time.time( ))
                #line: @M119 S0 @M120 S0 @M121 S0 @M122 S0 AS4 T9 C0
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

        
    def traslate_status(self, status):
    
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
        elif status == 'A':
            return 'CENTRAGGIO'
        elif status == 'B':
            return 'RILASCIATO'
        elif status == 'C':
            return 'NON_AUTENTICATO'
            
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
        # B - RILASCIATO       X       -      X       X       -       0     * Il recinto si disattiva da quando via TCP compare EM1
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
                                         
            logger.info("Position: (%s, %s, %s, %s), Status: %s (%s), Time: %s, Completed: %s" % (
                motor_yaw, 
                motor_front, 
                motor_rear_right,
                motor_rear_right, 
                self.traslate_status(self.canStatus), 
                self.canStatus,
                self.posTime, self.posProgress))
                
            self.oldStatus = self.canStatus
             
if __name__ == '__main__':

    logger = logging.getLogger('root')
    FORMAT = "[%(filename)s:%(lineno)3s - %(funcName)20s() ] %(message)s"
    logging.basicConfig(format=FORMAT, level=logging.INFO)

    tripod = ALMA_Tripod()

    # Called on process interruption. Set all pins to "Input" default mode.
    def endProcess(signalnum = None, handler = None):
        tripod.cleanup()
        
    # Install the exit handler
    signal.signal(signal.SIGINT, endProcess)
    
    # Start the CANOPEN server
    #tripod.start_canopen()

    # Avvio il server TCP
    reactor.listenTCP(10101, ALMA_Streamer_Factory(tripod))

    # Avvio il reattore!
    reactor.run()