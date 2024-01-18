# Agent Class
import CONFIG
from DataPoint import DataPoint


class Agent(DataPoint):

    def __init__(self, new_id=(CONFIG.NAME_AGENT + str(0).zfill(CONFIG.NAME_ZERO_FILL))):
        self.agent_id: str = new_id

        self.source_id: str = ""
        self.destination_id: str = ""

        self._transaction_id: str = ""
        self.my_id = self._transaction_id

        self.data: DataPoint = None

    @property
    def transaction_id(self):
        return self._transaction_id

    @transaction_id.setter
    def transaction_id(self, value):
        self.my_id = value
        self._transaction_id = value

    # function to reset agent
    def reset_assignment(self):
        # reset transaction_id
        self.transaction_id = ""
        self.my_id = ""
        self.data_id = ""

        # remove contained data
        self.data = None
