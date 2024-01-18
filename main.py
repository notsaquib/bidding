from loguru import logger
import asyncio

import CONFIG
import DataGenerator
from ExternalMarket import ExternalMarket
from TradingPlatform import Platform
from Machine import Machine


async def run():
    # create logger
    logger.add('log/log' + CONFIG.get_time_key())

    # instantiate platform and other modules
    platform = Platform()
    external_market = ExternalMarket(platform)
    external_market.register_in_platform()

    # generate data
    machine_jobs, suppliers_data = DataGenerator.generate_data()
    external_market.create_suppliers(data=suppliers_data)

    # create machines
    machines = []
    for _ in range(CONFIG.N_MACHINES):
        machine = Machine(platform)
        machine.register_in_platform()
        machines.append(machine)

    # add jobs to the machines
    for machine, job in machine_jobs:
        machines[machine].records_keeper.add_job(job)

    # generate schedules for jobs that were added to the machines
    for machine in machines:
        machine.scheduler.generate_schedules()

        # output schedules in console
        # machine.scheduler.print_schedules()
        pass

    # platform.run()


if __name__ == "__main__":
    asyncio.run(run())
