from dataclasses import dataclass, field

import CONFIG
from CONFIG import PRIORITY_DEFAULT, JobStatus, JobConstraint, get_time_key, create_time_list, ScheduleSrc, \
    ObjectiveGeneral, ObjectiveJob


# Datapoints are used inside Tables in Queues and always keyed by their my_id

# class for all machine datapoints
@dataclass
class DataPoint:
    my_id: str = "DefaultDataPoint"
    priority: int = PRIORITY_DEFAULT

    def __post_init__(self):
        self._my_id = self.my_id
        if not isinstance(self.my_id, str):
            raise ValueError('value not a string')

    @property
    def my_id(self):
        return self._my_id

    @my_id.setter
    def my_id(self, value):
        self._my_id = value


# class for a single job_id entry
@dataclass
class JobDataPoint(DataPoint):
    constraints: dict = field(default_factory=dict)
    objectives: list[ObjectiveJob] = field(default_factory=list)
    status: JobStatus = JobStatus.ENERGY_NEEDED

    # TODO: implement naming lists in CONFIG
    @property
    def time_ready(self):
        return self.constraints[JobConstraint.TIME_READY]

    @property
    def time_deadline(self):
        return self.constraints[JobConstraint.TIME_DEADLINE]

    @property
    def time_processing(self):
        return self.constraints[JobConstraint.TIME_PROCESSING]

    @property
    def energy_demand(self):
        return self.constraints[JobConstraint.ENERGY_DEMAND]

    # @property
    # def objectives(self):
    #     return self.objectives


# class for estimated list of prices
# this is used for estimation in machine to FCA agent and ECE
@dataclass
class EstimationDataPoint(DataPoint):
    timestamp: str = get_time_key()  # store the time of estimation
    periods_start: [str] = field(default_factory=list)
    prices: [float] = field(default_factory=list)

    def __post_init__(self):
        self.my_id = self.timestamp

    # create new EstimationDataPoint based on job_id data
    @staticmethod
    def create_from_job(job: JobDataPoint):
        estimation = EstimationDataPoint()
        estimation.priority = job.priority  # priority same as job_id priority
        estimation.periods_start = create_time_list(job.time_ready, job.time_deadline +
                                                    (CONFIG.MARGIN_PRICE_TIME * CONFIG.TIME_INTERVAL_UNIX))
        estimation.prices = [0.0 for i in range(len(estimation.periods_start))]
        estimation.my_id = job.my_id
        return estimation

    @staticmethod
    def create_from_interval(start: int, finish: int):
        estimation = EstimationDataPoint()
        estimation.priority = PRIORITY_DEFAULT
        estimation.periods_start = create_time_list(start, finish +
                                                    (CONFIG.MARGIN_PRICE_TIME * CONFIG.TIME_INTERVAL_UNIX))
        estimation.prices = [0.0 for i in range(len(estimation.periods_start))]
        return estimation

    def get_price(self, time):
        # TODO: ERROR if a date lies out of the range that was acquired from ECE
        index = self.periods_start.index(time)
        return self.prices[index]


# CLass for a single price estimation entry
@dataclass
class PriceDataPoint(DataPoint):
    period_start: str = ""
    _timestamps: [str] = field(default_factory=list)
    _prices: [float] = field(default_factory=list)

    def __post_init__(self):
        if not isinstance(self.period_start, str):
            raise ValueError('value not a string')
        self.my_id = self.period_start

    @property
    def timestamps(self):
        return self._timestamps

    @timestamps.setter
    def timestamps(self, value: str):
        self._timestamps.append(value)

    @property
    def prices(self):
        return self._prices

    @prices.setter
    def prices(self, value: str):
        self._prices.append(value)


# class for a single live agent entry in MachineAgentHandler
# used to shadow price and agent requests
@dataclass
class Shadow(DataPoint):
    reference_id: str = ""
    extra_id: str = ""
    data: DataPoint = None

    def __post_init__(self):
        if not isinstance(self.reference_id, str):
            self.reference_id = str(self.reference_id)
        self.my_id: str = self.reference_id


# class for schedule-slot
@dataclass
class ScheduleSlot:
    job_id: str = ""
    time_start: int = 0
    time_finish: int = 0
    cost: float = 0.0
    energy: float = 0.0

    def __str__(self):
        return f"slot: {self.job_id}: [start: {self.time_start}, finish: {self.time_finish}, " \
               f"energy: {self.energy}kWh, cost: {self.cost}] "


# class for Key Performance Indicators
class KPI:
    makespan: float = 0.0
    max_pastdue: float = 0.0
    sum_pastdue: float = 0.0
    num_pastdue: float = 0.0
    num_on_time: float = 0.0
    fraction_on_time: float = 0.0
    priority_job_pastdue: float = 0.0

    # method to relate general-objectives to KPI values
    def value(self, objective: ObjectiveGeneral) -> float:
        value = 0.0

        if objective is ObjectiveGeneral.MAKESPAN:
            value = self.makespan
        elif objective is ObjectiveGeneral.MAX_PASTDUE:
            value = self.max_pastdue
        elif objective is ObjectiveGeneral.SUM_PASTDUE:
            value = self.sum_pastdue

        return value

    def __str__(self):
        return f"Makespan: {self.makespan}, Maximum Past-due: {self.max_pastdue}, Sum Past-due: {self.sum_pastdue}," \
               f" No. Past-Due: {self.num_pastdue}, No. On-time: {self.num_on_time}," \
               f" Frac On-time: {self.fraction_on_time}, No. Priority-job Past-due: {self.priority_job_pastdue}"


# dataclass representing a schedule
@dataclass
class Schedule:
    # save which kind of scheduling created this schedule
    source: ScheduleSrc = ScheduleSrc.NONE
    # save the jobs inside this schedule
    jobs: [str] = field(default_factory=list)
    # total cost for this schedule
    total_cost: float = 0
    # key-performance-indicators
    kpi: KPI = KPI()
    # table of all schedule slots
    table: [ScheduleSlot] = field(default_factory=list)

    def add_slot(self, slot: ScheduleSlot):
        self.table.append(slot)
        self.jobs.append(slot.job_id)
        self.total_cost = sum([slot.cost for slot in self.table])

    def add_entry(self, job_id, start, finish, energy):
        self.jobs.append(job_id)
        self.total_cost = sum([slot.cost for slot in self.table])

        self.table.append(ScheduleSlot(job_id=job_id, time_start=start, time_finish=finish, energy=energy))

    def add_cost_entry(self, job_id: str, cost: float):
        # get index of job
        index = self.jobs.index(job_id)
        # add cost entry to job slot
        self.table[index].cost = cost
        # update schedule total cost
        self.total_cost = sum([slot.cost for slot in self.table])

    def get_time_start(self, job):
        index = self.jobs.index(job)
        return self.table[index].time_start

    def get_time_finish(self, job):
        index = self.jobs.index(job)
        return self.table[index].time_finish

    def get_cost(self, job):
        index = self.jobs.index(job)
        return self.table[index].cost

    def get_total_cost(self):
        return self.total_cost

    def __str__(self):
        # create header
        string = f"Schedule {self.source.value}\n"

        # append slots printout
        for slot in self.table:
            string += "\t" + str(slot) + "\n"

        # append costs and kpi
        string += f"\tKPI: {self.kpi}\n"
        string += f"\tTotal cost: {self.total_cost}\n"

        return string


# class to encapsulate a bidding object
@dataclass
class BidSlot(DataPoint):
    my_id = CONFIG.NAME_BID_SLOT + str(0).zfill(CONFIG.NAME_ZERO_FILL)
    source: str = ""
    slot: int = 0
    price: float = 0.0
    energy: float = 0.0


# class to encapsulate job bids
@dataclass
class BidJob(DataPoint):
    my_id: str = CONFIG.NAME_BID_JOB + str(0).zfill(CONFIG.NAME_ZERO_FILL)
    job_id: str = ""
    priority: int = CONFIG.PRIORITY_DEFAULT
    source_entity: str = ""
    source_strategy: str = ""
    bid_priority: int = 0
    bids: [BidSlot] = field(default_factory=list)

    def bids_from_schedule_slot(self, slot: ScheduleSlot, markup):
        time_list = create_time_list(time_start=slot.time_start, time_end=slot.time_finish)
        bid_counter = 0
        for index in range(len(time_list)):
            bid_counter += 1
            self.bids.append(BidSlot(my_id=self.my_id + CONFIG.NAME_BID_SLOT
                                     + str(bid_counter).zfill(CONFIG.NAME_ZERO_FILL),
                                     source=self.source_entity, energy=slot.energy, slot=time_list[index],
                                     price=slot.cost + (slot.cost*markup)))
