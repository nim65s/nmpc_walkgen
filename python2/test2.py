import numpy as np
import os, sys
import time
from walking_generator.visualization import Plotter
from walking_generator.combinedqp import NMPCGenerator
from walking_generator.interpolation import Interpolation

# import pyximport; pyximport.install()
from test_fct import fib

l = np.array([1,2,3])
print(l)
fib(200)
