from loguru import logger

# Machine sub-module classes
import CONFIG
from DataPoint import Shadow, DataPoint, EstimationDataPoint, BidJob
from MachineEventListener import MachineEvent
from PlatfromHashTable import PlatformHashTable
from CC import CC


# Agent handler Module
class MachineAgentHandler:
    # manage agent references
    def __init__(self, machine):
        # Agent_Handler_ID is the same as Machine_ID
        self.machine = machine

        # create agents hashtable
        self.shadow_requests_table = PlatformHashTable()
        self.shadow_agents_table = PlatformHashTable()

        # register FCC
        self.fcc: CC = None
        self.platform = None

        # TODO: extend references to other communication controllers

        # register Machine_Event_listener
        self.event_listener = machine.events_listener

    # functions to be called by the machine

    # function to request an agent
    def request_factory_agent(self, request_id, data: EstimationDataPoint):
        # create a shadow request for tracking
        new_request = Shadow(reference_id=request_id, data=data)
        self.shadow_requests_table.insert(datapoint=new_request)

        # log request
        logger.info(self.my_id + ' ' + f'factory agent request {request_id}')

        # request an agent from FCC and get a response
        self.fcc.machine_request_agent(machine_id=self.machine.my_id, request_id=request_id, priority=data.priority)

    def request_bidding_agent(self, request_id, data: BidJob):
        # create a shadow request for tracking
        new_request = Shadow(reference_id=request_id, data=data)
        self.shadow_requests_table.insert(datapoint=new_request)

        #  log request
        logger.info(self.my_id + ' ' + f'bidding agent request {request_id}')

        # request an agent from BCC and get a response
        self.fcc.machine_request_agent(machine_id=self.machine.my_id, request_id=request_id, priority=data.priority)

    # functions to be called by FCC

    def machine_grant_agent(self, request_id, transaction_id):
        # TODO: choose cc according to request_id

        # retrieve data related to request_id
        data = self.shadow_requests_table.remove(request_id).data

        # create shadow agent for agent tracking
        new_agent = Shadow(reference_id=transaction_id, extra_id=request_id, data=data)
        self.shadow_agents_table.insert(new_agent)

        # Set the FCC agent parameters
        self.platform.ai_set_source_id(transaction_id, self.my_id)
        self.platform.ai_set_destination_id(transaction_id, CONFIG.ID_ECE)
        self.platform.ai_set_data(transaction_id, data)

        logger.info(self.my_id + ' ' + f'agent transaction {transaction_id} granted by request {request_id}')

    # function to handle agent return from ECE
    def machine_receive_agent(self, transaction_id):
        # TODO: fix bug
        # get request_id that returned this data

        # get estimated data using transaction_id
        new_data = self.platform.ai_get_data(transaction_id)

        # get the original request_id from internal table
        # request_id = self.shadow_agents_table.find(transaction_id).extra_id

        # remove shadow agent entry related to the transaction_id, since it expired
        # self.shadow_agents_table.remove(transaction_id)

        # trigger new on_estimation_acquired event
        new_event = MachineEvent()
        new_event.data_id = new_data.my_id
        new_event.data = new_data

        # notify the EventHandler that a new price estimate is acquired
        self.machine.events_listener.on_estimation_acquired(new_event)

        logger.info(self.my_id + ' ' + f'agent return transaction {transaction_id}')

    # function to register agent handler with machine ID
    def register_platform_modules(self, machine):
        self.fcc = machine.platform.fcc
        # self.bcc = machine.platform.
        self.platform = machine.platform
        # TODO: register bcc

    # method to return entity_id
    @property
    def my_id(self):
        return self.machine.my_id
