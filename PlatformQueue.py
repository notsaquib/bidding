# Queue class implementation
from queue import Queue
from asyncio import PriorityQueue

import CONFIG
from DataPoint import Shadow


class PlatformQueue():
    def __init__(self, max_size=CONFIG.N_AGENTS):
        self.queue = PriorityQueue(max_size)

    def put(self, new_item):
        if isinstance(new_item, Shadow):
            priority = new_item.priority
        else:
            priority = CONFIG.PRIORITY_DEFAULT

        self.queue.put_nowait((priority, new_item))

    def get(self):
        data = self.queue.get_nowait()[1]
        return data

    def size(self):
        return self.queue.qsize()

    def is_empty(self):
        return self.queue.empty()

    def is_full(self):
        return self.queue.full()
