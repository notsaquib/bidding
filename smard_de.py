# API Documentation found at https://smard.api.bund.dev/
# Read wholesale price data from smard.de. Extensible through use of other filters.

import datetime
import requests
from typing import Union
from pytz import timezone

import CONFIG

# data is available in 1-week blocks, starting at epoch time (milliseconds) on Mondays, 00:00:00 (Berlin),
# therefore: determine most recent index
# NOTE: request for quarter hours is pretty much senseless (now) since data are interpolated from hourly data

# a few constants, could become variables if this is desirable
region = "DE"
resolution = "hour"
berlin = timezone('Europe/Berlin')

# excerpt of filters. All filters available in source above.
WHOLESALE = 4169        # 4169: Wholesale market price for region DE-LU
OVERALL_GEN = 122       # 122:  Forecasted overall generation
RENEWABLE_GEN = 5097    # 5097: Forecasted renewable generation
CONVENTIONAL_GEN = 715  # 715:  Forecasted conventional generation


def get_latest_date(filter_var=WHOLESALE):
    # get available timestamps. Filter is mandatory, could be changed according to requested data, though.
    timestamps_available = requests.get(
        f"https://www.smard.de/app/chart_data/{filter_var}/{region}/index_{resolution}.json"
        ).json()

    # select most current block
    latest_block = len(timestamps_available["timestamps"])-1

    # read most recent epoch time available
    return timestamps_available["timestamps"][latest_block]
 

def request_from_smard(filter_var, weekly_timestamp: Union[datetime.datetime, int]):
    # check first if weekly_timestamp has the correct format, if not, modify it
    if type(weekly_timestamp) is int and not weekly_timestamp%3600000:
        pass
    elif type(weekly_timestamp) is datetime.datetime:
        # determine the timestamp for the week's time
        weekly_timestamp = smard_timestamp(weekly_timestamp)
    else:
        raise ValueError("Timestamp format is not supported. Use allowed epoch times (msec) or datetime object.")

    # request data
    data = requests.get(
        f"https://www.smard.de/app/chart_data/{filter_var}/{region}/{filter_var}_"
        f"{region}_{resolution}_{weekly_timestamp}.json"
        ).json()

    # reformat data to dictionary
    # input format: dictionary with key "series": [[timestamp0, value0], [timestamp1, value1], ...]
    data_dict = {}
    for timestamps, values in data["series"]:
        # Since data are provided for the whole week, some timeslots have no values yet. Discard these.
        if values is None:
            break
        # perform conversion to epoch time and store values
        current_timestamp = (timestamps/1000)
        data_dict[current_timestamp] = values

    return data_dict


def get_wholesale_prices(weekly_timestamp = get_latest_date(filter_var=WHOLESALE), filter_var=WHOLESALE):
    # return most recent prices, unless past timeslot is specified
    return request_from_smard(filter_var, weekly_timestamp)


def get_source_composition(filter_source=RENEWABLE_GEN, filter_overall=OVERALL_GEN,
                           weekly_timestamp=get_latest_date(filter_var=OVERALL_GEN)):
    # first get overall generation
    overall_generation = request_from_smard(filter_overall, weekly_timestamp)
    # then get filtered generation
    filtered_generation = request_from_smard(filter_source, weekly_timestamp)
    # calculate fraction
    filtered_fraction = {}
    for ts in overall_generation.keys():
        filtered_fraction[ts] = filtered_generation[ts] / overall_generation[ts]

    return filtered_fraction


def smard_timestamp(epochtime):
    # determine the timestamp for the week's time
    # extract number of the week
    slot_timestamp = datetime.datetime.fromtimestamp(epochtime)
    year = slot_timestamp.astimezone(berlin).isocalendar().year
    week_no = slot_timestamp.astimezone(berlin).isocalendar().week

    # generate timestamp for Monday 00:00:00.0000 ('Europe/Berlin') for this week
    weekly_timestamp = int(berlin.localize(datetime.datetime.fromisocalendar(year, week_no, 1)).timestamp() * 1000)
    return weekly_timestamp


# print(get_source_composition(weekly_timestamp=datetime.datetime.strptime('20220203190000000000',CONFIG.TIME_KEY_FORMAT)))

# print(get_source_composition(CONVENTIONAL_SOURCES))
