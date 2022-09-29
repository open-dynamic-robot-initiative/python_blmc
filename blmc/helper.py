"""Some helper functions."""
import sys
import time


def get_time():
    if sys.platform == "win32":
        # On Windows, the best timer is time.clock
        return time.clock()
    else:
        # On Linux, clock does not return wall time but CPU time. use time.time
        # instead (which has much higher accuracy for linux than for windows,
        # so it is fine).
        return time.time()
