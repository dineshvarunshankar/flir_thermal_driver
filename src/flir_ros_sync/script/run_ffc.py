import os
from Boson_SDK import *
import time

myport = pyClient.Initialize(manualport="/dev/ttyACM0")
# run boson FFC
result = pyClient.bosonRunFFC()
print("FFC result: ", result)

pyClient.Close(myport)