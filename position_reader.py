__author__ = 'Luca Lucci'
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from twisted.internet import protocol, reactor
from twisted.protocols.basic import LineReceiver
import re, signal, logging
import plot



class ALMA_Config():

    def __init__(self):
        self.isLoginRequired = False


class ALMA_Stream_Protocol(LineReceiver):
    """Protocollo per la ricezione delle posizioni ALMA"""

    delimiter = '\n'
    def __init__(self, tripod):
        self.tripod = tripod
        self.command_timeout = None

    def lineLengthExceeded(self, line):
        print "length exceed"

    def lineReceived(self, data):
        #print 'Data received: {0}'.format(data)

        if self.tripod.simulation is True:
            self.tripod.line.append(data)

        self.tripod.stream_reader(data)

        if self.tripod.last_status != self.tripod.canStatus:
            self.tripod.last_status = self.tripod.canStatus

    def connectionMade(self):
        #self.tripod.plot.plot_init()
        self.tripod.plot.start()

    #    self.command_timeout = task.LoopingCall(self.tripod.update_output, self)
    #    self.command_timeout.start(1)


class ALMA_Streamer_Factory(protocol.ClientFactory):
    """Factory di gestione del protocollo di invio delle posizioni ALMA"""

    def __init__(self, tripod):
        self.tripod = tripod
        self.tripod.stream_factory = self

    def buildProtocol(self, addr):
        logger.info("Connection established with stream")
        self.tripod.almaStreamProtocol = ALMA_Stream_Protocol(self.tripod)
        return self.tripod.almaStreamProtocol

    def clientConnectionLost(self, connector, reason):
        logger.info("Connection lost. " % reason)

    def clientConnectionFailed(self, connector, reason):
        logger.info("Connection failed. " % reason)


class ALMA_Tripod():
    """Classe principale per la gestione del tripode.

    """

    def __init__(self):

        logger.info("Initializing ALMA_Tripod class")

        # Istanzio le classi
        self.config = ALMA_Config()
        self.plot = plot.ALMA_Plot(1, "Plot", 1)

        # All'avvio il sistema si trova nello stato 3, ATTIVO
        self.canStatus = '-1'
        self.last_status = self.canStatus
        self.last_response = ''
        self.userLoggedIn = False
        self.isAsyncError = False
        self.simulation = False

        self.almaControlProtocol = None
        self.md5sum = ''

        # Inizializza le strutture
        self.line = []
        self.time = 0
        self.roll = []
        self.pitch = []
        self.yaw = []

        self.posProgress = 0
        self.isInitialize = False

        # Compila i cercatori
        self.find_motor_position = re.compile('@M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) @M([^ ]*) S([^ ]*) AS([^ ]*) T([^ ]*) C([^ ]*)')

    def cleanup(self):
        logger.info("De-Initializing ALMA_Tripod class")

        self.plot.cleanup()

        # Chiude il reattore se attivo
        if reactor.running:
            logger.info("Stopping reactor")
            reactor.stop()

    def stream_reader(self, line):
        try:
            #line: @M119 S0 @M120 S0 @M121 S0 @M122 S0 AS4 T9.89 C0
            canopen_status = self.find_motor_position.search(line)

            self.time = 0.01

            if canopen_status is not None:
                self.plot.plot_update(self.time, float(canopen_status.group(2)), float(canopen_status.group(4)),
                                      float(canopen_status.group(6)), float(canopen_status.group(8)))
                #self.plot.plot_update(self.time, float(canopen_status.group(2)), float(canopen_status.group(4)),
                #                      float(canopen_status.group(6)), float(canopen_status.group(8)))

                if canopen_status:
                    # Se lo stato e' zero, vuol dire che c'e' una segnalazione pendente
                    if canopen_status.group(9) == '0' and not self.isAsyncError:
                        self.isAsyncError = True
                    elif canopen_status.group(9) == '8':
                        if self.simulation is False:
                            self.simulation = True
                    elif canopen_status.group(9) != '0':
                        self.simulation = False

                    self.canStatus = canopen_status.group(9)

                    self.posProgress = canopen_status.group(11)
        except Exception, e:
            logger.info(e.message)

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


if __name__ == '__main__':
    logger = logging.getLogger('root')
    FORMAT = "[%(filename)s:%(lineno)3s - %(funcName)20s() ] %(message)s"
    logging.basicConfig(format=FORMAT, level=logging.INFO)

    tripod = ALMA_Tripod()

    # Called on process interruption. Set all pins to "Input" default mode.
    def endProcess(signalnum=None, handler=None):
        tripod.cleanup()
        #sys.exit()

    # Install the exit handler
    signal.signal(signal.SIGINT, endProcess)


    # Avvio il server TCP

    reactor.connectTCP("192.168.1.117", 10101, ALMA_Streamer_Factory(tripod))
    #reactor.connectTCP("192.168.178.156", 10002, ALMA_Control_Factory(tripod))

    # Avvio il reattore!
    reactor.run()
