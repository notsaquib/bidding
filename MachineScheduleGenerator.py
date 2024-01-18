from datetime import datetime, timedelta
import matplotlib.pyplot as plt
from loguru import logger

import CONFIG
from CONFIG import JobConstraint, JobRecField
from DataPoint import KPI
from MachineEventListener import MachineEvent
from SchedulingStrategies import EmpiricalScheduling, PyomoScheduling


class MachineScheduleGenerator:
    def __init__(self, machine):
        self.machine = machine
        self.jobs_record = machine.records_keeper.jobs_record
        self.jobs = {}
        self.schedules = []

    def normalize_costs(self, x, x_min, x_max):
        # min-max feature scaling
        difference = x_max - x_min
        x_new = (x - x_min) / difference
        return x_new

    def print_schedules(self):
        # print schedules using a readable time format
        for schedule in self.schedules:
            print(schedule)

    def generate_schedules(self):
        self.jobs = self.create_dictionary()
        self.schedules.append(EmpiricalScheduling.first_in_first_out(self.jobs))
        self.schedules.append(EmpiricalScheduling.earliest_due_date_first(self.jobs))
        self.schedules.append(EmpiricalScheduling.last_in_first_out(self.jobs))
        self.schedules.append(EmpiricalScheduling.shortest_processing_time(self.jobs))
        self.schedules.extend(PyomoScheduling.opt_schedule(self.jobs, self.schedules[-1]))

        # generate Key-performance-indicators
        self.performance_indicators()

        # log schedules generated
        self.log_schedule_generation()

        # store generated schedules in record keeper
        self.machine.records_keeper.add_schedules(self.schedules)

        # create event of schedules generated
        self.machine.events_listener.on_schedules_generated(MachineEvent())

    # TODO: get rid of it and replace with direct referencing to record-keeper
    def create_dictionary(self):
        jobs = self.jobs_record.loc[:][JobRecField.DATA]
        prices_list = []
        job_dict = {}
        for job in jobs:
            estimate_dp = self.machine.records_keeper.get_job_prices(job.my_id)
            prices_list.append({job.my_id: estimate_dp.prices})
            job_dict.update({job.my_id: {
                JobConstraint.TIME_READY: job.time_ready,
                JobConstraint.TIME_DEADLINE: job.time_deadline,
                JobConstraint.TIME_PROCESSING: job.time_processing,
                JobConstraint.ENERGY_DEMAND: job.energy_demand,
                'PRIORITY': job.priority,
                'PRICES': estimate_dp
            }})
        return job_dict

    def performance_indicators(self):
        for schedule in self.schedules:
            for slot in schedule.table:
                cost = self.calculate_job_cost(start=slot.time_start, finish=slot.time_finish,
                                               energy_demand=self.machine.records_keeper.get_job_energy(slot.job_id))
                schedule.add_cost_entry(job_id=slot.job_id, cost=cost)

            # calculate KPI
            schedule.kpi = self.calculate_kpi(schedule)

    # helper method for calculating KPI for a schedule
    def calculate_kpi(self, schedule):
        kpi = KPI()
        kpi.makespan = (max(slot.time_finish for slot in schedule.table)
                        - min(slot.time_start for slot in schedule.table)) / CONFIG.TIME_INTERVAL_UNIX
        kpi.max_pastdue = max(max(0, slot.time_finish - self.jobs[slot.job_id][JobConstraint.TIME_DEADLINE])
                              for slot in schedule.table) / CONFIG.TIME_INTERVAL_UNIX
        kpi.sum_pastdue = sum(max(0, slot.time_finish - self.jobs[slot.job_id][JobConstraint.TIME_DEADLINE])
                              for slot in schedule.table) / CONFIG.TIME_INTERVAL_UNIX
        kpi.num_pastdue = sum(slot.time_finish > self.jobs[slot.job_id][JobConstraint.TIME_DEADLINE]
                              for slot in schedule.table)
        kpi.num_on_time = sum(slot.time_finish <= self.jobs[slot.job_id][JobConstraint.TIME_DEADLINE]
                              for slot in schedule.table)
        kpi.fraction_on_time = kpi.num_on_time / len(schedule.table)
        kpi.priority_job_pastdue = sum((slot.time_finish > self.jobs[slot.job_id][JobConstraint.TIME_DEADLINE]) *
                                       (CONFIG.PRIORITY_LEVELS - self.jobs[slot.job_id]['PRIORITY'])
                                       for slot in schedule.table)
        return kpi

    # helper method for calculating job_id cost
    def calculate_job_cost(self, start: int, finish: int, energy_demand: float):
        # calculating the cost of each job_id if it is scheduled as proposed
        cost = 0
        for slot in range(int(start), int(finish), CONFIG.TIME_INTERVAL_UNIX):
            # get price from record keeper
            price = self.machine.records_keeper.get_time_price(slot)

            # division / 1000 is subject to units: price [€ / MWh], energy_demand [kWh / slot]
            cost += price * energy_demand / 1000
        return cost

    def log_schedule_generation(self):
        for schedule in self.schedules:
            logger.info(self.machine.my_id + f": schedule generated: {schedule.source.value}")

    # Gantt Chart largely based on code from
    # https://jckantor.github.io/ND-Pyomo-Cookbook/notebooks/04.02-Machine-Bottleneck.html#example
    def plot_schedules(self):
        bw = 0.3
        for refschedule in self.schedules:
            schedule = self.schedules[refschedule]
            JOBS = self.jobs
            plt.figure(figsize=(12, 0.7 * (len(schedule.keys()))))
            idx = 0
            for j in sorted(JOBS.keys()):
                x = datetime.fromtimestamp(JOBS[j][JobConstraint.TIME_READY])
                y = datetime.fromtimestamp(JOBS[j][JobConstraint.TIME_DEADLINE])
                plt.fill_between([x, y], [idx - bw, idx - bw], [idx + bw, idx + bw], color='cyan', alpha=0.6)
                if j in schedule.keys():
                    x = datetime.fromtimestamp(schedule.get_time_start(j))
                    y = datetime.fromtimestamp(schedule.get_time_finish(j))
                    plt.fill_between([x, y], [idx - bw, idx - bw], [idx + bw, idx + bw], color='red', alpha=0.5)
                    plt.plot([x, y, y, x, x], [idx - bw, idx - bw, idx + bw, idx + bw, idx - bw], color='k')
                    plt.text(datetime.fromtimestamp((schedule.get_time_start(j) + schedule.get_time_finish(j)) / 2),
                             idx,
                             'Job ' + j, color='white', weight='bold',
                             horizontalalignment='center', verticalalignment='center')
                idx += 1

            plt.ylim(-0.5, idx - 0.5)
            plt.title(refschedule)
            plt.xlabel('Time')
            plt.ylabel('Jobs')
            plt.yticks(range(len(JOBS)), sorted(JOBS.keys()))
            plt.grid()
        plt.show()
