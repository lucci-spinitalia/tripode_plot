# -*- coding: utf-8 -*-

from twisted.internet import protocol
import threading
import logging

class Canopen(protocol.ProcessProtocol):
    """Gestione del server CANOPEN
    Implementa il wrapper per il controllo della comunicazione CANOPEN, che viene qui creato un un processo
    esterno, catturando opportunamente STDIN e STDOUT.
    """

    def __init__(self, tripod):

        logging.info("Initializing ALMA_Canopen class")

        self.data = ""
        self.last_command = ""
        self.is_sending_weight_correction = 0
        self.weight_value = 0L
        self.sender = "local"
        self.tripod = tripod
        self.is_sending_position = 0
        self.is_lowering = 0
        #self.VT_L = 1255360
        #self.AT_L = 1000
        self.VT_L = 600000
        self.AT_L = 500
        self.VT_R = 100000
        self.AT_R = 200
        self.relative_speed = 0.1

    def sendCommand(self, command, sender):

        if self.connected:
            self.transport.write("%s\n" % command)
            self.last_command = command
            self.sender = sender
            logging.info("To Canopen '%s\\n'" % command)

    def connectionMade(self):

        logging.info("ConnectionMade with CANOpen!")
        
        # Avvio il thread per la lettura delle posizioni
        self.tripod.isReading = True
        self.tripod.receiver_thread = threading.Thread(target=self.tripod.stream_reader)
        self.tripod.receiver_thread.setDaemon(True)
        self.tripod.receiver_thread.start()
        
    def outReceived(self, data):
        """Sono stati ricevuti bytes su STDOUT dal sotto-processo
        Il sotto-processo ha appena inviato su stdout delle informazioni, che rigiro a chi connesso via TCP"""

        # Se sto abbassando il tripode, invio prima tutti i comandi
        if self.is_lowering == 1:

            self.sendCommand("CT1 M120 P{} VM{} AM{}".format(
                318000,
                self.tripod.canopen.VT_L / 10,
                self.tripod.canopen.AT_L / 10
            ), "self")
            self.is_lowering = 2
            return

        elif self.is_lowering == 2:

            self.sendCommand("CT1 M121 P{} VM{} AM{}".format(
                318000,
                self.tripod.canopen.VT_L / 10,
                self.tripod.canopen.AT_L / 10
            ), "self")
            self.is_lowering = 3
            return

        elif self.is_lowering == 3:

            self.sendCommand("CT1 M122 P{} VM{} AM{}".format(
                318000,
                self.tripod.canopen.VT_L / 10,
                self.tripod.canopen.AT_L / 10
            ), "self")
            self.is_lowering = 0
            return

        # Se sto posizionando il tripode, invio prima tutti i comandi
        elif self.is_sending_position == 1:

            VT_L = int(self.tripod.canopen.VT_L / 100.0 * self.relative_speed)
            AT_L = int(self.tripod.canopen.AT_L / 100.0 * self.relative_speed)
            self.sendCommand("CT1 M120 P{} VM{} AM{}".format(self.tripod.kinematic.last_conversion_steps[0],
                                                             VT_L, AT_L), "self")
            self.is_sending_position = 2
            return

        elif self.is_sending_position == 2:

            VT_L = int(self.tripod.canopen.VT_L / 100.0 * self.relative_speed)
            AT_L = int(self.tripod.canopen.AT_L / 100.0 * self.relative_speed)
            self.sendCommand("CT1 M121 P{} VM{} AM{}".format(self.tripod.kinematic.last_conversion_steps[1],
                                                             VT_L, AT_L), "self")
            self.is_sending_position = 3
            return

        elif self.is_sending_position == 3:

            VT_L = int(self.tripod.canopen.VT_L / 100.0 * self.relative_speed)
            AT_L = int(self.tripod.canopen.AT_L / 100.0 * self.relative_speed)
            self.sendCommand("CT1 M122 P{} VM{} AM{} S".format(self.tripod.kinematic.last_conversion_steps[2],
                                                               VT_L, AT_L), "self")
            self.is_sending_position = 0
            self.sender = "remote"
            return

        # Se la simulazione è terminata chiudo il file di log
        if data[:6] == "OK CT4":
            self.tripod.last_sim_file.close()

        # Se il comando proviene dall'utente restituisce la risposta
        logging.info("From Canopen '%s'" % data.replace("\n", "\\n"))
        if self.sender == "remote":
            logging.info("  To TCP '%s'" % data.replace("\n", "\\n"))
            self.tripod.send_tcp_response(data)
        else:
            logging.info("  Private message")

        # Se viene mandata la conferma di presenza di motori, imposta il parametro di peso
        if self.is_sending_weight_correction == 2:
            self.sendCommand("PR5 M122 O60FB S008 T32s {:4X}".format(self.weight_value), "local")
            self.is_sending_weight_correction = 0
        elif self.is_sending_weight_correction == 1:
            self.sendCommand("PR5 M121 O60FB S008 T32s {:4X}".format(self.weight_value), "local")
            self.is_sending_weight_correction = 2
        elif data[:6] == "OK CT0":
            # Numero da inviare = -600000 -(Peso * 2000)
            # 0   -> -600000
            # 50  -> -700000
            # 100 -> -800000
            # 60FB 008 Signed 32 bit
            #    PR5 M120 O60FB S008 T32s 64F
            #    PR5 M121 O60FB S008 T32s 64F
            #    PR5 M122 O60FB S008 T32s 64F
            if self.tripod.antenna_weight > 0:
                self.weight_value = ((-600000 - (int(self.tripod.antenna_weight) * 2000)) & 0xffffffff)
                self.sendCommand("PR5 M120 O60FB S008 T32s {:4X}".format(self.weight_value), "local")
                self.is_sending_weight_correction = 1

    def errReceived(self, data):
        """Sono stati ricevuti bytes su STDERR dal sotto-processo
        Il sotto-processo ha appena inviato su stderr delle informazioni"""

        logging.info("errReceived! with %d bytes! %s" % (len(data), data))

    def inConnectionLost(self):

        logging.info("inConnectionLost! stdin is closed! (we probably did it)")

    def outConnectionLost(self):

        logging.info("outConnectionLost! The child closed their stdout!")
        
    def errConnectionLost(self):

        logging.info("errConnectionLost! The child closed their stderr.")

    def processExited(self, reason):

        if reason.value.exitCode:
            logging.info("processExited, exit code %d" % (reason.value.exitCode,))
        else:
            logging.info("processExited")
        # TODO: Per gestire CT6, al momento chiudo il programma
        #       In futuro dovrò eseguire lo shutdown
        self.tripod.cleanup()
        
    def processEnded(self, reason):

        if reason.value.exitCode:
            logging.info("processEnded, exit code %d" % (reason.value.exitCode,))
        else:
            logging.info("processEnded")

