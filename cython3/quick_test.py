import signal
import time
import numpy as np

class AlarmException(Exception):
    pass

def alarmHandler(signum, frame):
    raise AlarmException

def nonBlockingRawInput(previous,timeout):
    signal.signal(signal.SIGALRM, alarmHandler)
    signal.setitimer(signal.ITIMER_REAL,timeout,0.0)
    try:
        text = raw_input('new vel ? \n')
        signal.alarm(0)
        print("new vel !")
        l = String2Array(text)
        return l
    except AlarmException:
        print 'No new vel. Continuing...'
        return previous
    signal.signal(signal.SIGALRM, signal.SIG_IGN)

def String2Array(string):
    l = np.array(string.split(","), dtype='float')
    return l

i=0
timeout = 1e-3
l = np.zeros((1,3))
while True:
	print("iter: ",i, l)
	l = nonBlockingRawInput(l,timeout)
	time.sleep(2)
	i+=1
