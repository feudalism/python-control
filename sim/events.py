# -*- coding: utf-8 -*-

import numpy as np

def event(t, x):
    x_max = 5
    return np.abs(x)-x_max

event.terminal = True