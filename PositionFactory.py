# -*- coding: utf-8 -*-

from twisted.internet import protocol
import PositionProtocol

class PositionFactory(protocol.Factory):
    """Factory di gestione del protocollo di invio delle posizioni ALMA"""

    def __init__(self, tripod):
        # Devo impedire il collegamento multiplo!
        self.tripod = tripod
        self.tripod.position_factory = self
        self.numProtocols = 0
    
    def buildProtocol(self, addr):
        self.tripod.almaPositionProtocol = PositionProtocol.PositionProtocol(self.tripod)
        return self.tripod.almaPositionProtocol
