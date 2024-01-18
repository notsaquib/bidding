# class for temporary ECE
import random
from loguru import logger
import pandas as pd

import CONFIG
from CC import CC
from DataPoint import EstimationDataPoint, Shadow
from PlatformQueue import PlatformQueue
from PlatformEntity import PlatformEntity
from PlatfromHashTable import PlatformHashTable
import smard_de
import datetime


class ECE(PlatformEntity):
    def __init__(self, platform):
        # relate to relevant entities
        #self.my_id = CONFIG.ID_ECE
        self.platform = platform
        self.fcc: CC = None
        super().__init__(my_id=CONFIG.ID_ECE)

        self._request_id = 0
        self._estimate_id = 0

        # queue of completed estimations
        self.completed_estimates = PlatformQueue()

        # relate agent requests to data
        self.shadow_requests = PlatformHashTable()

        # relate entity_id to estimate_id
        self.shadow_estimates = PlatformHashTable()

        # create table ECE table
        self.price_estimates = {}

    # Get estimates from sources
    def get_estimates(self, estimate_id: str, data: EstimationDataPoint):
        # timestamp when estimation data were obtained
        data.timestamp = CONFIG.get_time_key()

        for slot in data.periods_start:
            week_stamp = smard_de.smard_timestamp(slot)
            # check whether data for needed timeslots has already been requested
            if week_stamp in self.price_estimates:
                pass
            else:
                # logging
                logger.info(self.my_id + ' ' + f'requesting data from smard.de for timestamp {week_stamp}')
                # request prices from smard
                self.price_estimates[week_stamp] = smard_de.get_wholesale_prices(week_stamp)

            # check if timeslot has data available
            if slot in self.price_estimates[week_stamp]:
                # insert data according to current period into prices list with same index
                data.prices[data.periods_start.index(slot)] = self.price_estimates[week_stamp][slot]
            else:
                # logging
                logger.info(self.my_id + ' ' + f'cannot provide data for timestamp {slot}')

        self.on_estimation_completed(estimate_id, data)

    # function for the FCC to ask for estimates
    def cc_service_function(self, transaction_id):
        # Get return machine my_id
        machine_id = self.platform.ai_get_source_id(transaction_id)

        # data should be an EstimateDataPoint
        data = self.platform.ai_get_data(transaction_id=transaction_id)

        # increment estimate_id and relate to entity_id
        estimate_shadow = Shadow(reference_id=self.estimate_id, extra_id=machine_id, data=data)
        self.shadow_estimates.insert(estimate_shadow)

        # logging
        logger.info(self.my_id + ' ' + f'FCC requests estimation transaction_id {transaction_id}')

        # pass the data along with the estimate_id for estimation
        self.get_estimates(estimate_shadow.my_id, data)

    # handle estimation completion
    def on_estimation_completed(self, estimate_id: str, data: EstimationDataPoint):
        # retrieve shadow related to estimate_id to get related entity_id
        shadow_estimate = self.shadow_estimates.remove(estimate_id)
        machine_id = shadow_estimate.extra_id

        # the new data is put in a shadow along with the entity_id and request_id
        shadow_request = Shadow(reference_id=self.request_id, extra_id=machine_id, data=data)

        self.shadow_requests.insert(shadow_request)

        # logging
        logger.info(self.my_id + ' ' + f'Estimation {estimate_id} completed')
        if CONFIG.LOGGING_CONSOLE_ALLOWED:
            print(f"estimation finished for machine: {machine_id}, estimation_id: {shadow_estimate.my_id}")

        self.fcc.service_request_agent(shadow_request.reference_id, priority=data.priority)

    # method for fcc to grant an agent
    def cc_grant_agent(self, request_id, transaction_id):
        # get data from shadow_requests
        shadow_request = self.shadow_requests.remove(request_id)

        self.platform.ai_set_source_id(transaction_id, self.my_id)
        self.platform.ai_set_destination_id(transaction_id, shadow_request.extra_id)
        self.platform.ai_set_data(transaction_id, shadow_request.data)

    # register with platform
    def register_in_platform(self):
        self.fcc = self.platform.fcc

    @property
    def request_id(self):
        self._request_id += 1
        return str(self._request_id)

    @property
    def estimate_id(self):
        self._estimate_id += 1
        return str(self._estimate_id)

    @request_id.setter
    def request_id(self, value):
        self._request_id = value

    @estimate_id.setter
    def estimate_id(self, value):
        self._estimate_id = value
