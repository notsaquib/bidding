# class for Platform entities
from abc import abstractmethod
from dataclasses import dataclass

import CONFIG


class PlatformEntity:
    def __init__(self, my_id=CONFIG.NAME_PLATFORM_ENTITY + str(0).zfill(CONFIG.NAME_ZERO_FILL)):
        self._my_id = my_id

    @property
    def my_id(self):
        return self._my_id

    @my_id.setter
    def my_id(self, value):
        self._my_id = value

    @abstractmethod
    def register_in_platform(self):
        raise NotImplementedError
