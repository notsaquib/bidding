import math
from dataclasses import dataclass
from loguru import logger

import CONFIG
from CONFIG import ObjectiveGeneral, ObjectiveEval, ObjectiveJob
from DataPoint import Schedule, BidJob
from MachineRecordKeeper import MachineRecordKeeper


# class for an eval_profile strategy object
@dataclass
class StrategyEvaluation:
    my_id: str
    objectives: [ObjectiveGeneral]
    eval_profile: [ObjectiveEval]

    # ?Also include custom nominal weights?

    def __post_init__(self):
        self.my_id = CONFIG.NAME_STRATEGY_EVALUATION + self.my_id


class StrategyBidding:
    def __init__(self, my_id:str="", strategizer=None, markup: float=0.0, num_bids: int=1,
                 eval_strategy: StrategyEvaluation=None):
        self.my_id: str = CONFIG.NAME_STRATEGY_BIDDING + my_id
        self.strategizer = strategizer  # MachineStrategyBlock
        self.markup: float = markup
        self.num_bids: int = num_bids
        self.eval_strategy: StrategyEvaluation = eval_strategy

    @classmethod
    def implement_strategy(cls):
        raise NotImplementedError

    @classmethod
    def take_feedback(cls):
        raise NotImplementedError


class MachineStrategyBlock:

    def __init__(self, machine):
        # attributes used to perform eval_profile
        self.machine = machine
        self.schedules = machine.records_keeper.schedules_record
        self.records: MachineRecordKeeper = machine.records_keeper
        self.objectives: [ObjectiveGeneral] = []
        self.strategies_evaluation = []
        self.strategies_bidding = []

        # attributes used to store eval_profile results
        self.schedules_eval: dict = {}

        # machine adopted eval_profile profile and weights (default from CONFIG)
        self.evaluation_profile = CONFIG.LIST_EVALUATION_PROFILE
        self.evaluation_weights_nominal = CONFIG.DICT_WEIGHTS_NOMINAL

        # machine nominal eval_profile strategy
        self.strategy_evaluation_nominal = StrategyEvaluation(my_id="Nominal", objectives=self.objectives,
                                                              eval_profile=self.evaluation_profile)
        self.strategy_bidding_nominal = self.create_strategy_bidding_nominal()

        # add machine nominal strategies to lists of strategies
        self.strategies_evaluation.append(self.strategy_evaluation_nominal)
        self.strategies_bidding.append(self.strategy_bidding_nominal)

    # method to evaluate schedules for a given eval_profile strategy
    def evaluate_strategy_all_schedules(self, strategy: StrategyEvaluation):
        # create empty dictionary to store evaluation results for strategy
        self.schedules_eval[strategy.my_id] = {}

        # loop over all schedules to evaluate them using the strategy
        for schedule in list(self.records.schedules_record.values()):
            # calculate schedule objective cost and scheduled jobs objective cost
            schedule_cost = self.evaluate_schedule(schedule=schedule, objectives=strategy.objectives,
                                                   evaluation_profile=strategy.eval_profile)
            jobs_cost = self.evaluate_scheduled_jobs(schedule=schedule)

            # check eval_profile profile for combined or separate costs
            if ObjectiveEval.JOB_INCLUSIVE in strategy.eval_profile :
                self.schedules_eval[strategy.my_id].update({schedule.source: schedule_cost + jobs_cost})
            else:
                self.schedules_eval[strategy.my_id].update({schedule.source: (schedule_cost, jobs_cost)})

        logger.info(self.machine.my_id + f": evaluation strategy: {strategy.my_id} implemented")

    # method to trigger schedules eval_profile only using machine nominal eval_profile strategy
    def evaluate_nominal_schedules(self):
        self.evaluate_strategy_all_schedules(strategy=self.strategy_evaluation_nominal)

    # method to evaluate all schedules using all eval_profile strategies
    def evaluate_strategies_schedules(self):
        [self.evaluate_strategy_all_schedules(strategy=strategy)
         for strategy in self.strategies_evaluation]

    # method to get eval_profile results for nominal strategy
    def get_schedules_eval_nominal(self):
        return self.schedules_eval[self.strategy_evaluation_nominal.my_id]

    # method to implement nominal eval_profile strategy
    def implement_strategies_bidding(self):
        [strategy.implement_strategy() for strategy in self.strategies_bidding]

        logger.info(self.machine.my_id + ": " + "all bidding strategies implemented")

    # method to set machine objectives
    def add_objectives(self, objectives):
        self.objectives.append(objectives)

    # method to set machine eval_profile strategies
    def add_strategy_evaluation(self, strategy: StrategyEvaluation):
        self.strategies_evaluation.append(strategy)

    # method to calculate the objective cost of ta schedule
    def evaluate_schedule(self, schedule: Schedule, objectives: list, evaluation_profile: ObjectiveEval) -> float:
        objective_cost = 0.0

        # check if objectives list is not empty and eval_profile profile is selective
        if objectives and (ObjectiveEval.GENERAL_SELECTIVE in evaluation_profile):
            objective_cost = sum([self.evaluation_weights_nominal[objective] * schedule.kpi.value(objective)
                                  * self.specific_weight(objective, objectives) for objective in objectives])
        else:  # else eval_profile is comprehensive
            objective_cost = sum([self.evaluation_weights_nominal[objective] * schedule.kpi.value(objective)
                                  * self.specific_weight(objective, objectives)
                                  for objective in CONFIG.LIST_GENERAL_OBJECTIVES_ALL])

        return objective_cost

    # method to calculate the objective cost of jobs as scheduled
    def evaluate_scheduled_jobs(self, schedule: Schedule) -> float:
        for slot in schedule.table:
            job = self.records.get_job(slot.job_id)
            # for each job in the schedule, calculate sum of all objective costs and then sum for all jobs
            return sum([sum([self.evaluation_weights_nominal[objective] *
                             self.value_obj_job(job_id=job, schedule=schedule, objective=objective)
                             for objective in self.records.get_job(job_id=job).objectives])
                        for job in schedule.jobs])

    def low_cost_strategy(self):
        # sort schedules for the lowest cost
        selection = sorted(self.schedules, key=lambda source: self.schedules[source].get_total_cost)
        # select the schedule with the lowest cost
        return selection

    # Helper functions

    # function to return value of general specified objectives
    @staticmethod
    def specific_weight(objective: ObjectiveGeneral, objectives: list) -> float:
        value = 1.0
        if objective in objectives:
            value = CONFIG.WEIGHT_OBJECTIVE_SPECIFIED
        return value

    # function to calculate priority factor based on PriorityProfile
    def priority_factor(self, priority: int) -> float:
        arithmatic_priority = CONFIG.PRIORITY_LEVELS - priority
        factor = 1.0

        if ObjectiveEval.PRIORITY_EXPONENTIAL in self.evaluation_profile:
            factor = math.exp(arithmatic_priority)
        elif ObjectiveEval.PRIORITY_LINEAR in self.evaluation_profile:
            factor = arithmatic_priority
        elif ObjectiveEval.PRIORITY_FACTORIAL in self.evaluation_profile:
            factor = math.prod([i for i in range(1, arithmatic_priority)])

        return factor

    # function to calculate value of job objective
    def value_obj_job(self, job_id: str, schedule: Schedule, objective: ObjectiveJob) -> float:
        value = 0.0
        # get job data related to objective
        if objective is ObjectiveJob.COST:
            value = schedule.get_cost(job=job_id)
        elif objective is ObjectiveJob.EARLY:
            value = schedule.get_time_start(job=job_id) - self.records.get_job(job_id=job_id).time_ready
        elif objective is ObjectiveJob.NO_TARDY:
            value = schedule.get_time_finish(job=job_id) - self.records.get_job(job_id=job_id).time_deadline
        return value

    # function to sort evaluated costs by least schedule cost
    def sort_eval_schedules(self):
        self.schedules_eval

    def create_strategy_bidding_nominal(self):
        class StrategyBiddingNominal(StrategyBidding):
            def __init__(self, strategizer: MachineStrategyBlock):
                super().__init__(my_id="Nominal", strategizer=strategizer, markup=CONFIG.DEFAULT_BIDDING_MARKUP,
                                 num_bids=CONFIG.DEFAULT_NUM_BIDS,
                                 eval_strategy=strategizer.strategy_evaluation_nominal)
                self.schedules = {}
                self.schedules_sorted_keys = []
                self.schedules_chosen = []
                self.counter_bid_job = 0
                self.bids = []

            def implement_strategy(cls):
                # get schedules in the relevant eval_profile strategy
                cls.schedules = cls.strategizer.schedules_eval[cls.eval_strategy.my_id]
                # sort evaluations by least cost
                cls.schedules_sorted_keys = sorted(cls.schedules, key=cls.schedules.get)
                # get schedules with the least cost
                cls.schedules_chosen = [cls.strategizer.records.get_schedule(schedule_id=cls.schedules_sorted_keys[i])
                                        for i in range(cls.num_bids)]
                # convert schedule to bids
                # loop over all bidding schedules
                bid_priority = 0
                for schedule in cls.schedules_chosen:
                    # loop over all slots in a schedule
                    for slot in schedule.table:
                        cls.counter_bid_job += 1
                        # create Job Bid
                        job_bid = BidJob(my_id=self.machine.my_id + schedule.source.value + CONFIG.NAME_BID_JOB +
                                               str(cls.counter_bid_job).zfill(CONFIG.NAME_ZERO_FILL),
                                         job_id=slot.job_id,
                                         priority=self.records.get_job(slot.job_id).priority, bid_priority=bid_priority,
                                         source_entity=self.machine.my_id, source_strategy=cls.my_id)
                        job_bid.bids_from_schedule_slot(slot=slot, markup=cls.markup)
                        # log bid generation for job
                        logger.info(self.machine.my_id + ": " + f"{cls.my_id} bid for job {slot.job_id} generated,"
                                                                f" bid-priority: {bid_priority},"
                                                                f" cost: {slot.cost + (slot.cost * cls.markup)}")
                        # save bids
                        cls.bids.append(job_bid)
                    # iterate bid priority so next bidding schedule is of lower priority
                    bid_priority += 1
                # log bidding generation
                logger.info(self.machine.my_id + ": " + f"bidding strategy {cls.my_id} implemented")

            def take_feedback(cls):
                pass

        # return nominal strategy
        return StrategyBiddingNominal(self)
