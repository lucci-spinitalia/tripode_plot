#!/usr/bin/env python
# -*- coding: utf-8 -*-

from twisted.internet import protocol, reactor, task
from time import sleep
from PositionProtocol import PositionProtocol
from PositionFactory import PositionFactory
from ControlFactory import ControlFactory
from Tripod import Tripod


import os, signal, logging, stat


class ALMA_Stream_Protocol(PositionProtocol):
    """Protocollo per l'invio delle posizioni ALMA"""

    def __init__(self, tripod):
        self.tripod = tripod
        PositionProtocol(self.tripod)

    def send_position(self):
        line = ''

        for motor_number in self.tripod.motor_address_list:
            line = line + "@M{} S{} ".format(motor_number, self.tripod.motorPos[motor_number])

        line = line + "AS{} T{} C{}".format(self.tripod.canStatus, self.tripod.posTime, self.tripod.OpProgress)

        self.transport.write("%s\n" % (line))

class ALMA_Streamer_Factory(protocol.Factory):
    """Factory di gestione del protocollo di invio delle posizioni ALMA"""

    def __init__(self, tripod):
        # Devo impedire il collegamento multiplo!
        self.tripod = tripod
        self.tripod.position_factory = self
        self.numProtocols = 0
    
    def buildProtocol(self, addr):
        self.tripod.almaPositionProtocol = ALMA_Stream_Protocol(self.tripod)
        return self.tripod.almaPositionProtocol
             
if __name__ == '__main__':

    logger = logging.getLogger('root')
    FORMAT = "[%(filename)s:%(lineno)3s - %(funcName)20s() ] %(message)s"
    logging.basicConfig(format=FORMAT, level=logging.INFO)

    tripod = Tripod()

    # Called on process interruption. Set all pins to "Input" default mode.
    def endProcess(signalnum = None, handler = None):
        tripod.cleanup()
        
    # Install the exit handler
    signal.signal(signal.SIGINT, endProcess)
    
    # Start the CANOPEN server
    tripod.start_canopen()

    # Avvio il server TCP
    reactor.listenTCP(10101, ALMA_Streamer_Factory(tripod))
    reactor.listenTCP(10102, ControlFactory(tripod))

    # Avvio il reattore!
    reactor.run()