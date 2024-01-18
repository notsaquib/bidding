# Configuration file
# imports
from enum import Enum
from datetime import datetime
from inputimeout import inputimeout, TimeoutOccurred
from configparser import ConfigParser
import numpy as np

from CONFIG_PROFILES import ConfigProfile, SimParam

# Read simulation parameters from a config.ini
config_sim = ConfigParser()
config_sim.read('CONFIG_PROFILES.ini')

# take input about chosen test profile
try:
    # test_profile = inputimeout(prompt=f'Choose test profile({list(ConfigProfile.__dict__.values())[1:4]}): ', timeout=5)
    test_profile = input(f'Choose test profile({list(ConfigProfile.__dict__.values())[1:4]}): ')
    sim_profile = config_sim[test_profile]
except (TimeoutOccurred, KeyError):
    print('Running profile: TEST')
    sim_profile = config_sim[ConfigProfile.TEST]

# Simulation Parameters
N_MACHINES = int(sim_profile[SimParam.NUM_MACHINES])
N_JOBS = int(sim_profile[SimParam.NUM_JOBS])
REF_DATETIME = str(sim_profile[SimParam.TIME_REF])
N_AGENTS = int(sim_profile[SimParam.NUM_AGENTS])  # Maximum number of agents created per Platform
N_SUPPLIERS = N_MACHINES
WEIGHT_OBJECTIVE_SPECIFIED = 10  # emphasis weight for specified objectives vs unspecified ones
DEFAULT_BIDDING_MARKUP = 1  # in %
DEFAULT_NUM_BIDS = 1

# Priority
# number of priority levels including zero. The lower priority value, the higher the priority.
PRIORITY_LEVELS = 3
# Default priority level
PRIORITY_DEFAULT = PRIORITY_LEVELS - 1  # Default priority is the lowest priority
PRIORITY_WEIGHT_NOMINAL = 1  # additional weight factored over the profile-calculated weight (see PriorityProfile)
# PRIORITY_PROFILE = PriorityProfile.EXPONENTIAL (see below)

# Supplier Parameters
PRICE_SUPPLY_MARGIN_MAX = 0.05  # maximum margin over ECE data that a supplier can charge
PRICE_SUPPLY_MARGIN_MIN = -0.05  # minimum margin below ECE data that a supplier can charge

# constants
# PlatformEntity IDs
ID_ECE = "ECE"
ID_FCC = "FCC"
ID_BCC = "BCC"

# Naming conventions
NAME_REQUEST_ESTIMATION = "ReqEst"
NAME_MACHINE = "M"
NAME_AGENT = "A"
NAME_JOB = "J"
NAME_SUPPLIER = "S"
NAME_BID_SLOT = "BS"
NAME_BID_JOB = "BJ"
NAME_PLATFORM_ENTITY = "PE"
NAME_STRATEGY_EVALUATION = "StrEval_"
NAME_STRATEGY_BIDDING = "StrBid_"
NAME_ZERO_FILL = int(np.log10(N_JOBS)) + 1 if N_JOBS >= N_MACHINES else int(np.log10(N_MACHINES)) + 1

# Time Parameters
TIME_KEY_FORMAT = "%Y%m%d%H%M%S%f"  # Formatting datetime instances
TIME_INTERVAL = 60  # time slot size in minutes
TIME_INTERVAL_UNIX = 60 * TIME_INTERVAL
TIME_INT_INTERVAL = 60  # every increment by 1-integer converted to minutes e.g. 60 mins in 1 (deprecated)
TIME_RELEASE_MAX = 50  # in units of TIME_INT_INTERVAL
TIME_DEADLINE_MAX = 100  # in units of TIME_INT_INTERVAL
REF_TIMESTAMP = int(datetime.strptime(REF_DATETIME, TIME_KEY_FORMAT).timestamp())
JOB_ENERGY_MIN = 50  # in kWh
JOB_ENERGY_MAX = 200  # in kWh
MARGIN_PRICE_TIME = 24  # in units of TIME_INT_INTERVAL used for extra price info after job_id due-time

# Initial number of buckets in all the Hash Tables used
N_INITIAL_CAPACITY = 50

# value of transaction_id to be interpreted as refused agent request
REFUSED_REQUEST = "0000"

# Return codes used by methods as a result status
METHOD_SUCCESS = 0
METHOD_FAILURE = 1

# Constant to control logging to the console
LOGGING_CONSOLE_ALLOWED = False


# Job Datapoint fields
# ENERGY_DEMAND, TIME_READY, TIME_DEADLINE, TIME_PROCESSING, PRIORITY
class JobRecField(Enum):
    JOB_ID = "JOB_ID"
    PRIORITY = "PRIORITY"
    STATUS = "STATUS"
    DATA = "DATA"


LIST_JOB_RECORD_FIELDS = [JobRecField.JOB_ID, JobRecField.PRIORITY, JobRecField.STATUS, JobRecField.DATA]


# Job constraints
class JobConstraint(Enum):
    TIME_READY = 'TIME_READY'
    TIME_DEADLINE = 'TIME_DEADLINE'
    TIME_PROCESSING = 'TIME_PROCESSING'
    ENERGY_DEMAND = 'ENERGY_DEMAND'


ENUM_JOB_CONSTRAINTS = [JobConstraint.TIME_READY, JobConstraint.TIME_DEADLINE, JobConstraint.TIME_PROCESSING,
                        JobConstraint.ENERGY_DEMAND]


# list(JobConstraint._member_map_.keys())


# job_id status enumeration
class JobStatus(Enum):
    ENERGY_NEEDED = "ENERGY_NEEDED"
    ENERGY_RESERVED = "ENERGY_RESERVED"


# ENUM_JOB_CONSTRAINTS = Enum('Job_Status', ['ENERGY_NEEDED', 'ENERGY_RESERVED'])
ENUM_JOB_STATUS = [JobStatus.ENERGY_NEEDED, JobStatus.ENERGY_RESERVED]


# objectives (minimize) for job
class ObjectiveJob(Enum):
    COST = "COST"
    EARLY = "EARLY"
    NO_TARDY = "NON_TARDY"


# objectives (minimize) for machine/strategies
class ObjectiveGeneral(Enum):
    MAKESPAN = "MAKESPAN"
    MAX_PASTDUE = "MAX_PASTDUE"
    SUM_PASTDUE = "SUM_PASTDUE"
    NUM_PASTDUE = "NUM_PASTDUE"
    COST = "COST"


# enumeration of objectives eval_profile method
# use only one type of prefix
class ObjectiveEval(Enum):
    GENERAL_SELECTIVE = "SELECTIVE"  # consider only specified objectives
    GENERAL_COMPREHENSIVE = "GENERAL_COMPREHENSIVE"  # consider all objectives with emphasis on specified objectives
    JOB_INCLUSIVE = "JOB_INCLUSIVE"  # include job objective as addition to general objective eval_profile
    JOB_EXCLUSIVE = "JOB_EXCLUSIVE"  # job objective eval_profile separate from general objective eval_profile
    PRIORITY_LINEAR = "PRIORITY_LINEAR"  # increasing priority increases weight linearly
    PRIORITY_EXPONENTIAL = "PRIORITY_EXPONENTIAL"  # increasing priority increases weight with exponentially
    PRIORITY_FACTORIAL = "PRIORITY_FACTORIAL"  # increasing weight priority increases weight by weight-factorial


# Dict of nominal objectives weights before specialization
DICT_WEIGHTS_JOB_NOMINAL = {ObjectiveJob.COST: 1, ObjectiveJob.EARLY: 20, ObjectiveJob.NO_TARDY: 20}
DICT_WEIGHTS_GENERAL_NOMINAL = {ObjectiveGeneral.COST: 1, ObjectiveGeneral.MAKESPAN: 10,
                                ObjectiveGeneral.MAX_PASTDUE: 10, ObjectiveGeneral.SUM_PASTDUE: 10,
                                ObjectiveGeneral.NUM_PASTDUE: 20}
DICT_WEIGHTS_NOMINAL = DICT_WEIGHTS_GENERAL_NOMINAL
DICT_WEIGHTS_NOMINAL.update(DICT_WEIGHTS_JOB_NOMINAL)

# Evaluation profile to be used
LIST_EVALUATION_PROFILE = [ObjectiveEval.GENERAL_COMPREHENSIVE, ObjectiveEval.JOB_INCLUSIVE,
                           ObjectiveEval.PRIORITY_EXPONENTIAL]

# list of all possible objectives
LIST_JOB_OBJECTIVES_ALL = list(ObjectiveJob)
LIST_GENERAL_OBJECTIVES_ALL = list(ObjectiveGeneral)

# Job Price estimation fields
LIST_PRICE_ESTIMATION = ['PERIOD_START', 'TIMESTAMP/PRICE']


# Schedule types
class ScheduleSrc(Enum):
    FIFO = 'FIFO'  # First-in, First-out
    EDD = 'EDD'  # Earliest due-date first
    LIFO = 'LIFO'  # Last-in, First-out
    SPT = 'SPT'  # Shortest processing-time
    PYOMO = 'PYOMO'  # Pyomo optimal model
    NONE = 'NONE'  # source not specified


# Useful Methods

# Function to return current time for key generation
def get_time_key():
    now = datetime.now()
    return now.strftime(TIME_KEY_FORMAT)


# Function to convert time integer from time string
def time_str_to_int(time_string: str):
    # convert relevant times to datetime instances
    ref_time = datetime.strptime(REF_DATETIME, TIME_KEY_FORMAT)
    time = datetime.strptime(time_string, TIME_KEY_FORMAT)

    # calculate time difference and get timedelta instance
    time_delta = time - ref_time

    # convert timedelta instance to integer value
    time_int = int((time_delta.total_seconds() / 60) / TIME_INT_INTERVAL)

    return time_int


# Function to create list of times between time and end time using TIME_INTERVAL
def create_time_list(time_start, time_end):
    time_list = []

    delta_intervals = int((time_end - time_start) / TIME_INTERVAL_UNIX)
    for i in range(delta_intervals):
        time_list.append(time_start + (i * TIME_INTERVAL_UNIX))
    return time_list


print('CONFIG imported')
