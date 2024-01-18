# class for Machine submodule: Record Keeper
# Keeps the records and logs needed for strategies and all decision-making
import pandas as pd

import CONFIG
from DataPoint import JobDataPoint, PriceDataPoint, EstimationDataPoint, Shadow, ScheduleSrc
from MachineEventListener import MachineEvent
from PlatfromHashTable import PlatformHashTable


class MachineRecordKeeper():
    def __init__(self, machine):
        self.machine = machine

        # record of added jobs
        self.jobs_record = pd.DataFrame(columns=CONFIG.LIST_JOB_RECORD_FIELDS)
        self.jobs_index = {}
        self.prices_record = PlatformHashTable()
        self.schedules_record = {}
        self.price_estimates = {}

    def add_job(self, new_job_datapoint: JobDataPoint):
        # create new job_id entry from JobDataPoint to insert into record
        job_data = self.job_datapoint_to_record_list(new_job_datapoint)
        # index for fast referencing using jobs_index hashtable and insert in jobs_record
        job_index = len(self.jobs_record)
        # insert new job_id entry in the dataframe and the hashtable
        self.jobs_record.loc[job_index] = job_data
        self.jobs_index[new_job_datapoint.my_id] = job_index

        # create new job_added event
        new_event = MachineEvent(my_id=new_job_datapoint.my_id, data_id=new_job_datapoint.my_id, data=new_job_datapoint)

        # notify machine event listener
        self.machine.events_listener.on_job_added(new_event)

    def add_prices(self, new_prices: EstimationDataPoint, job_id: str):
        # new_prices should be of type EstimationDatapoint
        timestamp = new_prices.timestamp
        # times = new_prices.periods_start
        prices = new_prices.prices

        # insert price datapoint in record
        self.prices_record.insert(new_prices)

        # insert prices in collective dictionary
        times = new_prices.periods_start
        for time in times:
            if time in self.price_estimates.keys():
                pass
            else:
                self.price_estimates[time] = prices[times.index(time)]

        # for j in range(len(new_prices)):
        #     # check if price period exists already
        #     if new_prices.periods_start[j] in self.prices_record['PERIOD_START']:
        #         # add new timestamp and price
        #         self.prices_record['DATAPOINT', new_prices.period_starts[j]].timestamps.append(timestamp)
        #         self.prices_record['DATAPOINT', new_prices.period_starts[j]].prices.append(new_prices.prices[j])
        #     else:
        #         new_price = [new_prices.periods_start[j]].append(timestamp).append(new_prices.prices[j])
        #         prices_data_i = PriceDataPoint(new_price)
        #         self.prices_record.append(vars[prices_data_i].append(prices_data_i))

        # create price added event
        new_event = MachineEvent(my_id=timestamp, data_id=job_id, data=new_prices)

        # notify machine event listener
        self.machine.events_listener.on_estimation_added(new_event)

    # methods to get price data

    # method to get jobs from record
    def get_job(self, job_id: str) -> JobDataPoint:
        # get index from jobs_index_hashtable
        index = self.jobs_index[job_id]

        # use index to get JobDataPoint from jobs_record
        return self.jobs_record.loc[index][CONFIG.JobRecField.DATA]

    # method to get price data between start and finish time
    def get_interval_prices(self, time_start: int, time_finish: int):
        time_list = CONFIG.create_time_list(time_start=time_start, time_end=time_finish)
        return self.price_estimates[time_list]

    # method to get prices from records
    def get_job_prices(self, job_id:str) -> EstimationDataPoint:
        return self.prices_record.find(job_id)

    # method to return price estimate at certain time
    def get_time_price(self, time: int) -> float:
        try:
            return self.price_estimates[time]
        except KeyError:
            miss_estimate = EstimationDataPoint.create_from_interval(start=max(self.price_estimates.keys()),
                                                                     finish=time)
            miss_estimate.my_id = "MissEst"+str(time)
            new_event = MachineEvent(my_id=CONFIG.get_time_key(), data=miss_estimate)
            self.machine.events_listener.on_missing_estimation(new_event)
            return 0.0

    # method to get job_id energy demand
    def get_job_energy(self, job_id: str):
        return self.get_job(job_id=job_id).energy_demand

    # methods for schedule data
    def add_schedules(self, schedules):
        for schedule in schedules:
            self.schedules_record[schedule.source] = schedule

    def get_schedule(self, schedule_id: ScheduleSrc):
        return self.schedules_record[schedule_id]

    # Helper methods

    # method to convert JobDataPoint object to record entry
    @staticmethod
    def job_datapoint_to_record_list(job_datapoint: JobDataPoint):
        my_id = job_datapoint.my_id
        priority = job_datapoint.priority
        status = job_datapoint.status
        return pd.Series([my_id, priority, status, job_datapoint], index=CONFIG.LIST_JOB_RECORD_FIELDS)

    # method for inserting estimated prices in the record
    def insert_price_entry(self, timestamp, time, price):
        # check if period estimation already exists
        if self.prices_record.find(time) is None:
            # create new entry
            price_entry = PriceDataPoint(period_start=time)
            price_entry.timestamps.append(timestamp)
            price_entry.prices.append(price)
        else:
            # add new timestamped price to existing entry
            price_entry = self.prices_record.find(time)
            price_entry.timestamps(timestamp)
            price_entry.prices(price)
