#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
from realmaze import run

if __name__ == "__main__":

    move = run()
    time.sleep(1.0)
    move.start()
    while move.goal_check() and move.error_stop():
        move.loop()
    print "goal and return"
    if move.error_stop():
        move.back()