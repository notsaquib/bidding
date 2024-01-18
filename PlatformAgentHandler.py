from loguru import logger

import CONFIG
from Agent import Agent
from DataPoint import DataPoint, Shadow
from PlatformQueue import PlatformQueue
from PlatfromHashTable import PlatformHashTable
import TradingPlatform


class PlatformAgentHandler:
    def __init__(self, platform: TradingPlatform):
        # reference to platform
        self.platform = platform

        # agents queue
        self.agents_queue = PlatformQueue()
        self.generate_agents()

        # Transactions & deployed agents hashtable
        self.deployed_agents_table = PlatformHashTable()

        # Hashtable for tracking relevant CC
        self.shadow_agent_cc = PlatformHashTable()

    # TODO: CCs to reserve agents before get agent ?

    # function to be called be CC to reserve an agent
    def reserve_agent(self, cc_id: str, transaction_id: str):
        if self.agents_queue.is_empty():
            deployment_success = False
        else:
            # reference agent to respective cc using transaction_id
            shadow_agent = Shadow(reference_id=transaction_id, data=cc_id)
            self.shadow_agent_cc.insert(shadow_agent)

            # deploy agent
            agent = self.agents_queue.get()
            agent.transaction_id = transaction_id
            self.deployed_agents_table.insert(agent)

            deployment_success = True

        return deployment_success

    # method to handle end of transaction
    def return_agent(self, transaction_id: str):
        # remove agent from deployed agents table
        agent = self.deployed_agents_table.remove(transaction_id)

        # Log end of transaction
        logger.info(f"Transaction {transaction_id} ended")

        # reset agent parameters
        agent.reset_assignment()

        # push agent to agents queue
        self.agents_queue.put(agent)

    # Agent interface functions to be called by outside entities
    def agent_set_data(self, transaction_id: str, data: DataPoint):
        # TODO: handle non-existent or expired transaction_id
        # get data and set agent
        agent = self.deployed_agents_table.find(transaction_id)
        agent.data = data

        # Agent is ready to be forwarded
        # TODO: Separate function to forward agent
        # get destination ID
        destination_id = agent.destination_id

        # get related CC
        cc_id = self.shadow_agent_cc.find(transaction_id)

        # TODO: handle different CCs
        self.platform.fcc.forward_agent(transaction_id=transaction_id,
                                        destination_id=destination_id)

    def agent_get_data(self, transaction_id: str):
        # TODO: handle non-existent or expired transaction_id
        data = self.deployed_agents_table.find(transaction_id).data

        # Agent ready on data request or change
        self.return_agent(transaction_id)

        return data

    def agent_set_source_id(self, transaction_id: str, source_id: str):
        self.deployed_agents_table.find(transaction_id).source_id = source_id

    def agent_set_destination_id(self, transaction_id: str, destination_id: str):
        self.deployed_agents_table.find(transaction_id).destination_id = destination_id

    def agent_get_source_id(self, transaction_id: str):
        return self.deployed_agents_table.find(transaction_id).source_id

    # Helper Methods

    # method for agents generation
    def generate_agents(self):
        for i in range(CONFIG.N_AGENTS):
            # Generate Agent ID
            agent_name = CONFIG.NAME_AGENT + str(i).zfill(CONFIG.NAME_ZERO_FILL)
            # Create Agent instance
            new_agent = Agent(new_id=agent_name)
            # Save Agent in Queue
            self.agents_queue.put(new_agent)
