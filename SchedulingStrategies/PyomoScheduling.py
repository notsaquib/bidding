# Solver based optimal scheduling based on examples found at
# https://jckantor.github.io/ND-Pyomo-Cookbook/notebooks/04.02-Machine-Bottleneck.html#pyomo-model
# with additions
import CONFIG
from CONFIG import JobConstraint, ScheduleSrc
from pyomo.environ import *
from pyomo.gdp import *
from DataPoint import Schedule


def opt_schedule(JOBS, order_start: Schedule):
    # create model
    m = ConcreteModel()

    # NOTE: The introduction of resolution is necessary to limit the decision space of the solver to a reasonable range.
    # factor to get hourly resolution
    resolution = CONFIG.TIME_INT_INTERVAL * CONFIG.TIME_INTERVAL

    # index set to simplify notation
    m.J = Set(initialize=JOBS.keys())
    m.PAIRS = Set(initialize=m.J * m.J, dimen=2, filter=lambda m, j, k: j < k)

    # upper bounds on how long it would take to process all jobs
    tmax = (max([JOBS[j][JobConstraint.TIME_READY] for j in m.J]) +
            sum([JOBS[j][JobConstraint.TIME_PROCESSING] for j in m.J])) / resolution
    tmin = min([JOBS[j][JobConstraint.TIME_READY] for j in m.J]) / resolution

    # initialize the m.time variable with a 'best guess'
    def initialize_m_start(m, j):
        return order_start.get_time_start(j)/resolution

    # decision variables
    m.start = Var(m.J, domain=PositiveIntegers, bounds=(tmin, tmax), initialize=initialize_m_start)
    m.pastdue = Var(m.J, domain=PositiveIntegers, bounds=(tmin, tmax))
    m.early = Var(m.J, domain=PositiveIntegers, bounds=(tmin, tmax))

    # additional decision variables for use in the objecive
    m.makespan = Var(domain=PositiveIntegers, bounds=(tmin, tmax))
    m.maxpastdue = Var(domain=PositiveIntegers, bounds=(tmin, tmax))
    m.ispastdue = Var(m.J, domain=Binary)

    # objective function
    m.OBJ1 = Objective(expr=sum([m.pastdue[j] for j in m.J]), sense=minimize)
    m.OBJ2= Objective(expr=sum([m.early[j] for j in m.J]), sense=minimize)
    m.OBJ3= Objective(expr=m.makespan, sense=minimize)

    objectives = []
    objectives.append(m.OBJ1)
    objectives.append(m.OBJ2)
    objectives.append(m.OBJ3)

    # constraints
    m.c1 = Constraint(m.J, rule=lambda m, j: m.start[j] >= JOBS[j][JobConstraint.TIME_READY] / resolution)
    m.c2 = Constraint(m.J, rule=lambda m, j: m.start[j] + JOBS[j][JobConstraint.TIME_PROCESSING] / resolution +
                      m.early[j] == JOBS[j][JobConstraint.TIME_DEADLINE] / resolution + m.pastdue[j])
    m.c3 = Disjunction(m.PAIRS, rule=lambda m, j, k:
                       [m.start[j] + JOBS[j][JobConstraint.TIME_PROCESSING] / resolution <= m.start[k],
                        m.start[k] + JOBS[k][JobConstraint.TIME_PROCESSING] / resolution <= m.start[j]])

    m.c4 = Constraint(m.J, rule=lambda m, j: m.pastdue[j] <= m.maxpastdue)
    m.c5 = Constraint(m.J, rule=lambda m, j: m.start[j] + JOBS[j][JobConstraint.TIME_PROCESSING] / resolution
                      <= m.makespan * resolution)
    m.c6 = Constraint(m.J, rule=lambda m, j: m.pastdue[j] <= tmax * m.ispastdue[j])

    # deactivate all objectives
    for obj in objectives:
        obj.deactivate()

    SCHEDULES = []

    for i, obj in enumerate(objectives):
        # activate objective
        obj.activate()

        # solve model
        TransformationFactory('gdp.hull').apply_to(m)
        SolverFactory('glpk').solve(m).write()

        schedule = Schedule(source=ScheduleSrc.PYOMO)

        for j in m.J:
            schedule.add_entry(j, m.start[j]() * resolution,
                               m.start[j]() * resolution + JOBS[j][JobConstraint.TIME_PROCESSING],
                               JOBS[j][JobConstraint.ENERGY_DEMAND])

        SCHEDULES.append(schedule)
        obj.deactivate()
    return SCHEDULES
