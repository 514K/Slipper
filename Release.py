from math import exp
from random import uniform, random, randint
import numpy as np

import re, subprocess
import xml.etree.ElementTree as ET


def f(vars : np.ndarray):   # forwatd dynamics function goes here
    if len(vars) == 5:
        # print(vars[0])
        if type(vars[0]) == np.float64 and type(vars[1]) == np.float64 and vars[0] >= 10000 and vars[0] <= 100000 and vars[1] >= 10000 and vars[1] <= 100000 and type(vars[2]) == np.float64 and type(vars[3]) == np.float64 and type(vars[4]) == np.float64 and vars[2] >= 0 and vars[2] <= 1 and vars[3] >= 0 and vars[3] <= 1 and vars[4] >= 0 and vars[4] <= 1:
            # edit translational_stiffness
            with open("ToyLandingModel_activeAFO_copy.osim") as file:
                mOsimDoc = file.read()

                # edit vec1 bushing
                mOsimDoc = re.sub("<translational_stiffness>\d* \d* \d*</translational_stiffness><!-- kostil1 -->", "<translational_stiffness>" + str(int(vars[0])) + " " + str(int(vars[0])) + " " + str(int(vars[0])) + "</translational_stiffness><!-- kostil1 -->", mOsimDoc)

                # edit vec2 bushing
                mOsimDoc = re.sub("<translational_stiffness>\d* \d* \d*</translational_stiffness><!-- kostil2 -->", "<translational_stiffness>" + str(int(vars[1])) + " " + str(int(vars[1])) + " " + str(int(vars[1])) + "</translational_stiffness><!-- kostil2 -->", mOsimDoc)
            file = open("ToyLandingModel_activeAFO_copy.osim", "w")
            file.write(mOsimDoc)
            file.close()

            # edit 0.3; 0.35; 0.4 dots of actuator
            mXmlDoc = ET.parse("ActiveAFO_Edited.xml")
            rootXmlDoc = mXmlDoc.getroot()
            element = rootXmlDoc[0][0][0][6]
            element[6][1].text = str(vars[2])
            element[7][1].text = str(vars[3])
            element[8][1].text = str(vars[4])
            mXmlDoc.write("ActiveAFO_Edited.xml")

            # start ForwardDynamics
            CallOpenSimCMD = subprocess.call(["opensim-cmd", "run-tool", "test.xml"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


            # analyze
            maxAngle = 0.0
            with open("SolveFromPy_states_degrees.mot", "r") as file:
                mStoDoc = file.readlines()[14:]
                for i in mStoDoc:
                    element = i.split("\t")
                    # print(element[31])
                    if float(element[31]) > maxAngle:
                        maxAngle = float(element[31])
                    
                    # print("time : " + element[0] + "; subtalar_angle_r/value : " + element[31])

                return maxAngle
        else:
            return "b"
    else:
        return "a"


def generate(x):    # function for new point generation
    y = np.array([uniform(10000, 100000), uniform(10000, 100000), uniform(0, 1), uniform(0, 1), uniform(0, 1)])
    return y


def generate1(x):   # another function for new point generation
    k=1
    while k == 1:
        y = np.array([randint(-10000, 10000), randint(-10000, 10000), uniform(-0.1, 0.1), uniform(-0.1, 0.1),
                    uniform(-0.1, 0.1)])
        a = x - y
        mm = 10000
        mmx = 100000
        mm1 = 0
        mmx1 = 1
        print("da")
        if a[0] > mmx or a[0] < mm or a[1] > mmx or a[1] < mm or a[2] > mmx1 or a[2] < mm1 or a[3] > mmx1 or a[3] < mm1 or a[4] > mmx1 or a[4] < mm1:

            continue
        else: 
            k=0
            return a


x0 =np.array([40000, 40000, 0.8, 0.8, 0.8]) # initial state
E = 25     # angle
Tmax = 1   # start of iterations
Tmin = 0.1     # end of iterations

c = 0.7    # coefficient

t = Tmax
x = x0
while t > Tmin:

    print(x, t)
    xt = generate1(x)   # creating new point

    St = np.linalg.norm(xt[0:2])    # curr norm of stiffness
    At = np.linalg.norm(xt[2:])     # curr norm of activation
    S = np.linalg.norm(x[0:2])  # old norm of stiffness
    A = np.linalg.norm(x[2:])   # old norm of activation
    if f(xt) < E and (St < S and At < A):    # if new point is better we accept it
        t = t * c
        S = St
        A = At
        x = xt
        continue
    elif f(xt) < E and (St < S or At < A):  # if new point is not better we randomly choose it
        probability = exp(- abs(At-A) / t)   # calculating probability of choosing
        #   print(probability)
        if random() < probability:
            t = t * c
            x = xt
    if f(xt) > E:
        t = t * c
    #   if the angle is more than 25 we generate new point


print(x, t)
