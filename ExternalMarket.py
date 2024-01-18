import random
from abc import ABC
from enum import Enum
from loguru import logger

import CONFIG
import TradingPlatform
from DataPoint import EstimationDataPoint, BidSlot
from MachineAgentHandler import MachineAgentHandler
from MachineEventListener import MachineEventListener, MachineEvent
from PlatformEntity import PlatformEntity


# class for enumerating entries of external market data generation
class ExtMarketData(Enum):
    LIST_ENERGY = "LIST_ENERGY"
    TIME_SUPPLY_PERIOD_FINISH = "TIME_SUPPLY_PERIOD_FINISH"


# External market player generation and implementation
class ExternalMarket(PlatformEntity):
    def __init__(self, platform: TradingPlatform):
        self.my_id = "ExternalMarket"
        self.platform = platform
        self.fcc: PlatformEntity = None
        self.bcc: PlatformEntity = None

        self.suppliers = []
        self.events_listener = MachineEventListener(self)
        self.agents_handler = MachineAgentHandler(self)

        self.data_energy = []
        self.data_supply_period_start = CONFIG.REF_TIMESTAMP
        self.data_supply_period_finish = 0
        self.data_prices: dict = {}  # EstimationDataPoint = None

    # function to take supplier data and generate requests for market estimations
    # determine beginning and end time for energy supply period
    # determine a random, consistent daily schedule for energy supply for each supplier
    # what distribution of energy supply and related supplier repetitions needed
    def create_suppliers(self, data: dict):
        # save data
        self.data_energy = data[ExtMarketData.LIST_ENERGY]
        self.data_supply_period_finish = data[ExtMarketData.TIME_SUPPLY_PERIOD_FINISH] \
                                         + (CONFIG.MARGIN_PRICE_TIME * CONFIG.TIME_INTERVAL_UNIX)

        # retrieve market estimations from ECE
        # create EstimationDataPoint
        estimate = EstimationDataPoint.create_from_interval(self.data_supply_period_start,
                                                            self.data_supply_period_finish)
        estimate.my_id = "Supplier_Create"
        request_id = "Supplier_Request"

        # log data request
        logger.info("External market price estimation requested")
        self.agents_handler.request_factory_agent(request_id=request_id, data=estimate)

    # function to generate a supplier based on data from the data generator
    def on_estimation_acquired(self, event: MachineEvent):
        # log data request fulfillment
        logger.info("External market price estimation acquired")

        # prices should be of type EstimationDatapoint
        estimates = event.data
        self.data_prices = {estimates.periods_start[i]: estimates.prices[i] for i in range(len(estimates.prices))}

        # create suppliers based on energy
        # each supplier provides energy at all the time slots for a given energy
        # price offers by each supplier vary on around +-5% by supplier
        for i in range(len(self.data_energy)):
            # create supplier with unique_id
            margin = round(random.uniform(CONFIG.PRICE_SUPPLY_MARGIN_MIN, CONFIG.PRICE_SUPPLY_MARGIN_MAX), 4)
            supplier = Supplier(market=self, energy=self.data_energy[i], margin=margin)
            supplier.register_with_external_market(my_id=CONFIG.NAME_SUPPLIER + str(i + 1).zfill(CONFIG.NAME_ZERO_FILL))
            self.suppliers.append(supplier)

            # loop for bid generation in supplier
            for j in range(len(estimates.periods_start)):
                supplier.add_bid(time=estimates.periods_start[j], price=estimates.prices[j])

    # register related services
    def register_in_platform(self):
        self.platform.register_external_entity(self)
        self.fcc = self.platform.fcc
        # self.bcc = self.platform.bcc
        self.agents_handler.register_platform_modules(self)


# class for suppliers
class Supplier:
    def __init__(self, market, energy, margin):
        self.my_id = CONFIG.NAME_SUPPLIER + str(0).zfill(CONFIG.NAME_ZERO_FILL)
        self.market = market
        self.energy = energy
        self.pricing = 1 + margin  # deviation from nominal pricing
        # self.time_generation: ScheduleSlot = None  # supply availability time in a day
        self.bids = []
        self.bid_counter = 0

    def add_bid(self, time, price):
        self.bid_counter += 1
        self.bids.append(BidSlot(my_id=self.my_id + CONFIG.NAME_BID_SLOT
                                       + str(self.bid_counter).zfill(CONFIG.NAME_ZERO_FILL),
                                 source=self.my_id, energy=self.energy, slot=time,
                                 price=price * self.pricing))

    # register useful components of the external markets
    def register_with_external_market(self, my_id: str):
        self.my_id = my_id
