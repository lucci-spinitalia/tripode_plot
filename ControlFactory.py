# -*- coding: utf-8 -*-

from twisted.internet import protocol
from ControlProtocol import ControlProtocol


class ControlFactory(protocol.Factory):
    """Factory di gestione del protocollo di controllo ALMA"""
    
    def __init__(self, tripod):
        # Devo impedire il collegamento multiplo!
        self.tripod = tripod
        self.tripod.control_factory = self
        self.tripod.isClientConnected = False
        self.tripod.connectedAddress = ""

    def buildProtocol(self, addr):
        self.tripod.almaControlProtocol = ControlProtocol(self.tripod)
        self.tripod.connectedAddress = addr
        return self.tripod.almaControlProtocol
