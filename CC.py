from loguru import logger

# Communication Controller Class inherits from Communication controller class
import CONFIG
from DataPoint import Shadow, DataPoint
from PlatformQueue import PlatformQueue
from PlatfromHashTable import PlatformHashTable
from PlatformEntity import PlatformEntity


# General Communication Controller Class
class CC:
    def __init__(self, platform, my_id="CC"):
        self.my_id = my_id
        self.platform = platform
        self.service: PlatformEntity = None
        self.agents_handler = None

        # Create hash tables and Queues
        # save unfulfilled agent requests by machine
        self.machine_requests_queue = PlatformQueue()

        # save unfulfilled agent requests by service
        self.service_requests_queue = PlatformQueue()

        # table to track agents deployed
        self.shadow_agents_table = PlatformHashTable()

        # event loop for running internal coroutines
        # self.loop: asyncio.AbstractEventLoop = asyncio.get_event_loop()

    # TODO: monitor resources and requests before assigning agents

    # function to listen to request addition events
    def on_new_agent_requested(self):
        # TODO: (Real-time Scheduling) manage granting of agents to either machine or ECE
        while not self.machine_requests_queue.is_empty():
            # retrieve request from requests queue
            request_shadow = self.machine_requests_queue.get()
            request_id = request_shadow.my_id
            machine_id = request_shadow.extra_id
            # Attempt to grant agent
            granted_flag, transaction_id = self.prepare_agent(entity_id=machine_id, request_id=request_id)
            if granted_flag:
                # grant agent
                self.machine_grant_agent(machine_id=machine_id, request_id=request_id, transaction_id=transaction_id)
                logger.info(self.my_id + ' ' + f'agent request {request_id} granted to {machine_id}')
            else:
                # return request to queue
                self.machine_requests_queue.put(request_shadow)
        while not self.service_requests_queue.is_empty():
            request_shadow = self.service_requests_queue.get()
            request_id = request_shadow.my_id
            # Attempt to grant agent
            granted_flag, transaction_id = self.prepare_agent(entity_id=self.service.my_id, request_id=request_id)
            if granted_flag:
                # grant agent
                self.service_grant_agent(request_id=request_id, transaction_id=transaction_id)
                logger.info(self.my_id + ' ' + f'agent request {request_id} granted to {self.service.my_id}')
            else:
                # return request to queue
                self.service_requests_queue.put(request_shadow)

    # function called by PlatformAgentHandler to forward agent to destination
    def forward_agent(self, transaction_id: str, destination_id: str):
        if destination_id is self.service.my_id:
            self.service.cc_service_function(transaction_id)
        else:
            self.platform.mi_machine_return_agent(machine_id=destination_id, transaction_id=transaction_id)

        logger.info(self.my_id + ' ' + f'agent transaction {transaction_id} forwarded to {destination_id}')

    # functions for CC-to-machine communications

    # function called by machine to request an agent
    def machine_request_agent(self, machine_id, request_id, priority=CONFIG.PRIORITY_DEFAULT):
        # put request in queue
        self.machine_requests_queue.put(Shadow(reference_id=request_id, extra_id=machine_id, priority=priority))

        # log to console
        if CONFIG.LOGGING_CONSOLE_ALLOWED:
            print(f"FCA request received: {request_id}")

        # log request
        logger.info(self.my_id + ' ' + f'agent requested by machine {machine_id}')

        # call CC events listener
        self.on_new_agent_requested()

    # function called by CC to grant agent to machine
    def machine_grant_agent(self, machine_id: str, request_id: str, transaction_id: str):
        # grant agent to machine
        self.platform.mi_machine_grant_agent(machine_id=machine_id,
                                             request_id=request_id, transaction_id=transaction_id)

    # Functions for communication with ECE

    # function for ECE to request agent
    def service_request_agent(self, request_id: str, priority=CONFIG.PRIORITY_DEFAULT):
        self.service_requests_queue.put(Shadow(reference_id=request_id, extra_id=CONFIG.ID_ECE, priority=priority))

        # log request by service
        logger.info(self.my_id + ' ' + f'agent requested by service {self.service.my_id}')

        self.on_new_agent_requested()

    def service_grant_agent(self, request_id: str, transaction_id: str):
        # grant agent to machine
        self.service.cc_grant_agent(request_id=request_id, transaction_id=transaction_id)

    # reserve agent to grant a request
    def prepare_agent(self, entity_id: str, request_id: str):
        # generate transaction_id
        transaction_id = self.generate_transaction_id(entity_id=entity_id, request_id=request_id)

        # get agent from agents queue
        deployment_status = self.agents_handler.reserve_agent(cc_id=self.my_id, transaction_id=transaction_id)

        # Relate agent to machine using shadow agent
        shadow_agent = Shadow(reference_id=transaction_id, data=entity_id)
        self.shadow_agents_table.insert(shadow_agent)

        logger.info(self.my_id + ' ' + f'agent request {request_id} prepared for {entity_id}')

        return deployment_status, transaction_id

    # method to generate transaction_id
    @staticmethod
    def generate_transaction_id(entity_id: str, request_id: str):
        # TODO: find suitable generation method
        return entity_id + str(request_id)

    # method to register relevant entities
    def register_in_platform(self):
        self.agents_handler = self.platform.agents_handler


# ===============================================================================================================
# ===============================================================================================================


class FCC(CC):
    def __init__(self, platform):
        CC.__init__(self, platform, my_id=CONFIG.ID_FCC)

    # method to register relevant entities
    def register_in_platform(self):
        CC.register_in_platform(self)
        self.service = self.platform.ece


class BCC(CC):
    def __init__(self, platform):
        CC.__init__(self, platform, my_id=CONFIG.ID_BCC)

    # method to register relevant entities
    def register_in_platform(self):
        CC.register_in_platform(self)
        self.service = self.bidding_module.market
