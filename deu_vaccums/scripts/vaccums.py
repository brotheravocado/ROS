#! /usr/bin/env python


import math
import time

from function import func



class run:


    def __init__(self):
        print("[----------------------------------------]")
        print("[-------------Welcome Vaccums------------]")
        print("[----------------------------------------]")
        self.function_fu = func()
        self.function_fu.start()
        self.started_time = time.time()

    def end_time(self):
        last_time=time.time()
        print "end time:",last_time-self.started_time

    def measurement_area(self):
        self.function_fu.cleaned_area()


if __name__ == "__main__":
    move = run()
    move.end_time()
    move.measurement_area()
