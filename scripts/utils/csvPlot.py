from matplotlib import pyplot
from csv import reader
from dateutil import parser

with open('/home/pi/Swarm/RoboDev/Controllers/PID/pid.csv', 'r') as fIn:
    dataIn = list(reader(fIn))

# with open('/home/hunter/devel/Swarm/Rev1-0/temp_dev/Algorithms/EKF/ekf_output.csv', 'r') as fOut:
#     dataOut = list(reader(fOut))

targs = [i[1] for i in dataIn[1::]]
meas = [i[2] for i in dataIn[1::]]
comm = [i[3] for i in dataIn[1::]]
err = [i[4] for i in dataIn[1::]]

pyplot.title('Data changes over Time')
pyplot.xlabel('Time/secs')
pyplot.ylabel('Data')

pyplot.plot(range(len(targs)), targs,'r')
pyplot.plot(range(len(targs)), meas,'b')
pyplot.plot(range(len(targs)), comm,'g')
pyplot.xlim(0,len(targs))
pyplot.ylim(-4,4)
pyplot.show()
