# Class for machine event
from dataclasses import dataclass
from loguru import logger

import CONFIG
from DataPoint import DataPoint


@dataclass
class MachineEvent(DataPoint):
    timestamp: str = CONFIG.get_time_key()
    data_id: str = ""
    data: DataPoint = None

    def __post_init__(self):
        self.my_id = self.timestamp

    def __str__(self):
        return self.timestamp + f": {self.data_id}"


# Machine Event Listener
# contains necessary methods to handle machine events
class MachineEventListener:
    def __init__(self, machine):
        self.machine = machine
        self.log_to_console = CONFIG.LOGGING_CONSOLE_ALLOWED

    # event triggered when a new job is added
    def on_job_added(self, machine_event: MachineEvent):
        # log event to console
        if self.log_to_console:
            print("New Job Added")
            print(machine_event)

        logger.info(self.machine.my_id + ' ' + f'new job_id {machine_event.data_id} added')

        # pass event data to machine for response
        # add relevant listeners here too
        self.machine.on_job_added(machine_event)

    # event triggered when time slot energy cost estimation is missing
    def on_missing_estimation(self, machine_event: MachineEvent):
        logger.info(self.machine.my_id + ' ' + f'price estimation {machine_event.data_id} missing')
        self.machine.on_missing_estimation(machine_event)

    # event triggered when a new estimation is returned from the ECE
    def on_estimation_acquired(self, machine_event: MachineEvent):
        # log event to console
        if self.log_to_console:
            print("Price Estimation Acquired")
            print(machine_event)

        logger.info(self.machine.my_id + ' ' + f'price estimation {machine_event.data_id} acquired')
        self.machine.on_estimation_acquired(machine_event)

    # event triggered when a new estimation is added to the record keeper
    def on_estimation_added(self, machine_event: MachineEvent):
        # log event to console
        if self.log_to_console:
            print("Price Estimation added")
            print(machine_event)

        logger.info(self.machine.my_id + ' ' + f'price estimation {machine_event.data_id} added')
        self.machine.on_estimation_added(machine_event)

    # event triggered when schedules are generated
    def on_schedules_generated(self, machine_event: MachineEvent):
        logger.info(self.machine.my_id + ' ' + f'schedules generated')
        self.machine.on_schedules_generated(machine_event)

    def on_bid_request(self, machine_event: MachineEvent):
        # log event to console
        if self.log_to_console:
            print("New Bid Added")
            print(machine_event)

    def on_bid_response_acquired(self, machine_event: MachineEvent):
        # log event to console
        if self.log_to_console:
            print("Bid Response Acquired")
            print(machine_event)
