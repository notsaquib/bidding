# main machine class
# all machine submodules and intelligence happen here
import CONFIG
from DataPoint import EstimationDataPoint
from DataPoint import Shadow
from MachineAgentHandler import MachineAgentHandler
from MachineEventListener import MachineEventListener, MachineEvent
from MachineRecordKeeper import MachineRecordKeeper
from MachineScheduleGenerator import MachineScheduleGenerator
from MachineStrategyBlock import MachineStrategyBlock
from PlatfromHashTable import PlatformHashTable
from PlatformEntity import PlatformEntity


class Machine(PlatformEntity):
    def __init__(self, platform):
        # register platform
        self.platform = platform

        # Declare machine sub-modules
        self.events_listener = MachineEventListener(self)
        self.agents_handler = MachineAgentHandler(self)
        self.records_keeper = MachineRecordKeeper(self)
        self.scheduler = MachineScheduleGenerator(self)
        self.bidder = MachineStrategyBlock(self)

        # Tables for shadowing agent requests
        self.price_requests_table = PlatformHashTable()

        # request_id count
        self._request_id = 0

    # event response functions

    # function to respond to job_id addition event
    def on_job_added(self, machine_event: MachineEvent):
        # Fetch data to be requested (should get job_id datapoint)
        job = machine_event.data

        # create new estimation datapoint
        new_estimate_datapoint = EstimationDataPoint.create_from_job(job)

        # create a new request_id
        current_request_id = CONFIG.NAME_REQUEST_ESTIMATION + self.request_id

        # add the job_id to the shadow_jobs_table
        shadow_job = Shadow(reference_id=job.my_id, extra_id=current_request_id)

        # insert in HashTable
        self.price_requests_table.insert(shadow_job)

        # request agent for estimation
        # take back request_id for tracking & logging
        self.agents_handler.request_factory_agent(request_id=current_request_id, data=new_estimate_datapoint)

    # function to respond to missing price estimation event
    def on_missing_estimation(self, machine_event: MachineEvent):
        # Fetch data to be requested
        estimate_dp = machine_event.data

        # create a new request_id
        current_request_id = CONFIG.NAME_REQUEST_ESTIMATION + self.request_id

        # add the job_id to the shadow_jobs_table
        shadow_data = Shadow(reference_id=estimate_dp.my_id, extra_id=current_request_id)

        # insert in HashTable
        self.price_requests_table.insert(shadow_data)

        # request agent for estimation
        # take back request_id for tracking & logging
        self.agents_handler.request_factory_agent(request_id=current_request_id, data=estimate_dp)

    # function to respond to price estimation acquired event
    def on_estimation_acquired(self, machine_event: MachineEvent):
        # remove request_id from HashTable (data_id should be job_id)
        shadow_job = self.price_requests_table.remove(machine_event.data_id)
        job_id = shadow_job.my_id

        # prices should be of type EstimationDatapoint
        prices_datapoint = machine_event.data

        # record new price
        self.records_keeper.add_prices(prices_datapoint, job_id=job_id)

    def on_estimation_added(self, machine_event):
        # TODO: Add job_id to record of non-scheduled jobs
        pass

    def on_schedules_generated(self, machine_event: MachineEvent):
        self.bidder.evaluate_nominal_schedules()
        self.bidder.implement_strategies_bidding()

    def on_bids_generated(self, machine_event: MachineEvent):
        print("End here")
        pass

    # function to register machine with platform
    def register_in_platform(self):
        self.my_id = self.platform.get_machine_id(self)

        # pass platform (part of self) to agent handler
        self.agents_handler.register_platform_modules(self)

    # Helper Methods

    @property
    def request_id(self):
        self._request_id += 1
        return str(self._request_id)
