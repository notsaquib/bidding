# Empirical scheduling based on examples found at
# https://jckantor.github.io/ND-Pyomo-Cookbook/notebooks/04.02-Machine-Bottleneck.html#empirical-scheduling
import CONFIG
from CONFIG import JobConstraint, ScheduleSrc
from DataPoint import Schedule

def ordered(jobs, order):
    """Schedule a dictionary of jobs on a single machine in a specified order."""
    schedule = Schedule(source=ScheduleSrc.FIFO)
    start = 0
    finish = 0
    for job in order:
        start = max(jobs[job][JobConstraint.TIME_READY], finish)
        finish = start + jobs[job][JobConstraint.TIME_PROCESSING]
        energy = jobs[job][JobConstraint.ENERGY_DEMAND]
        schedule.add_entry(job, start, finish, energy)
    return schedule


def first_in_first_out(jobs):
    order_by_release = sorted(jobs, key=lambda job: jobs[job][JobConstraint.TIME_READY])
    return ordered(jobs, order_by_release)


def earliest_due_date_first(jobs):
    schedule = Schedule(source=ScheduleSrc.EDD)
    unfinished_jobs = set(jobs.keys())
    start = 0
    while len(unfinished_jobs) > 0:
        start = max(start, min(jobs[job][JobConstraint.TIME_READY] for job in unfinished_jobs))
        edd = {job: jobs[job][JobConstraint.TIME_DEADLINE] for job in unfinished_jobs if
               jobs[job][JobConstraint.TIME_READY] <= start}
        job = min(edd, key=edd.get)
        finish = start + jobs[job][JobConstraint.TIME_PROCESSING]
        unfinished_jobs.remove(job)
        energy = jobs[job][JobConstraint.ENERGY_DEMAND]
        schedule.add_entry(job, start, finish, energy)
        start = finish
    return schedule


def last_in_first_out(jobs):
    schedule = Schedule(source=ScheduleSrc.LIFO)
    unfinished_jobs = set(jobs.keys())
    start = 0
    while len(unfinished_jobs) > 0:
        start = max(start, min(jobs[job][JobConstraint.TIME_READY] for job in unfinished_jobs))
        lifo = {job: jobs[job][JobConstraint.TIME_READY] for job in unfinished_jobs if
                jobs[job][JobConstraint.TIME_READY] <= start}
        job = max(lifo, key=lifo.get)
        finish = start + jobs[job][JobConstraint.TIME_PROCESSING]
        unfinished_jobs.remove(job)
        energy = jobs[job][JobConstraint.ENERGY_DEMAND]
        schedule.add_entry(job, start, finish, energy)
        start = finish
    return schedule


def shortest_processing_time(jobs):
    schedule = Schedule(source=ScheduleSrc.SPT)
    unfinished_jobs = set(jobs.keys())
    start = 0
    while len(unfinished_jobs) > 0:
        start = max(start, min(jobs[job][JobConstraint.TIME_READY] for job in unfinished_jobs))
        spt = {job: jobs[job][JobConstraint.TIME_PROCESSING] for job in unfinished_jobs if
               jobs[job][JobConstraint.TIME_READY] <= start}
        job = min(spt, key=spt.get)
        finish = start + jobs[job][JobConstraint.TIME_PROCESSING]
        unfinished_jobs.remove(job)
        energy = jobs[job][JobConstraint.ENERGY_DEMAND]
        schedule.add_entry(job, start, finish, energy)
        start = finish
    return schedule
