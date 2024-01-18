# class for the all-encompassing platform XD
import asyncio

import CONFIG
from CC import FCC, BCC
from DataPoint import DataPoint
from ECE import ECE
from PlatformAgentHandler import PlatformAgentHandler
from PlatformEntity import PlatformEntity
from PlatfromHashTable import PlatformHashTable
import PlatformBiddingModule


class Platform:
    def __init__(self):
        # Agent Handler
        self.agents_handler = PlatformAgentHandler(self)
        # self.bidding_module = PlatformBiddingModule(self)

        # create platform modules
        self.fcc = FCC(self)
        self.ece = ECE(self)
        self.bcc = BCC(self)

        # register platform components
        self.ece.register_in_platform()

        # register communication controllers
        self.fcc.register_in_platform()

        # create machines HashTable
        self.machine_id = 0  # counter for machine IDs
        self.machines = PlatformHashTable()

    # run the TradingPlatform from main
    def run(self):
        self.fcc.run()

    # Machine interface functions for calling machines
    def mi_machine_grant_agent(self, machine_id, request_id, transaction_id):
        machine = self.machines.find(machine_id)
        machine.machine_grant_agent(request_id, transaction_id)

    def mi_machine_return_agent(self, machine_id, transaction_id):
        machine = self.machines.find(machine_id)
        machine.machine_receive_agent(transaction_id)

    # Agent interface functions
    def ai_set_data(self, transaction_id: str, data: DataPoint):
        self.agents_handler.agent_set_data(transaction_id, data)

    def ai_get_data(self, transaction_id: str):
        return self.agents_handler.agent_get_data(transaction_id)

    def ai_set_source_id(self, transaction_id: str, source_id: str):
        self.agents_handler.agent_set_source_id(transaction_id, source_id)

    def ai_get_source_id(self, transaction_id: str):
        return self.agents_handler.agent_get_source_id(transaction_id)

    def ai_set_destination_id(self, transaction_id: str, destination_id: str):
        self.agents_handler.agent_set_destination_id(transaction_id, destination_id)

    # Helper function used for giving new machines an ID
    def get_machine_id(self, new_machine):
        # generate new entity_id string with formatting
        self.machine_id += 1
        machine_id = CONFIG.NAME_MACHINE + str(self.machine_id).zfill(CONFIG.NAME_ZERO_FILL)

        # set entity_id before inserting into HashTable
        new_machine.my_id = machine_id

        # add as external entity
        self.register_external_entity(entity=new_machine)

        # return entity_id to machine
        return machine_id

    def register_external_entity(self, entity: PlatformEntity):
        # add to machines table
        self.machines.insert(entity.agents_handler)
