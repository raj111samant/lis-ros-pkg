'''Calibrate the transmission ratios for the arm cables
To take data, uncomment mode = "take_data", start roscore, then run WAMinterface/ROS/WAMServerROS.py, then run this script, then run WAMinterface/WAMinterfacelib/socketwamif (as root!!) when prompted to do so.
To compute transmission ratios, uncomment mode = "compute_ratios"'''

#mode = "take_data"
mode = "compute_ratios"

import wamik
import scipy
import time
import cPickle as pickle
import pdb
import sys

#pretty-print a 4x4 scipy matrix
def ppmat4tostr(mat):
    printstr = ''
    for i in range(4):
        printstr += ' '.join(['%2.3f' % x for x in mat[i,:].tolist()[0]])+'\n'
    return printstr

#pretty-print a list of doubles to a string
def ppdoublelisttostr(list):
    return ' '.join(['%2.3f' % x for x in list])


#pretty-print a list of ints to a string
def ppintlisttostr(list):
    return ' '.join([str(x) for x in list])


#pretty-print a vector in nx1 scipy matrix form
def ppscipyvecttostr(mat):
    return ' '.join(['%2.3f' % x for x in mat.transpose().tolist()[0]])

#convert from joint0 frame to base frame
joint0_to_base_mat = scipy.matrix([[1,0,0,.22],
                                   [0,1,0,.14],
                                   [0,0,1,.346],
                                   [0,0,0,1]], dtype=float)

#create a 4x4 scipy matrix that rotates about z by angle (rad)
def zrotmat(angle):
    return scipy.matrix([[math.cos(angle), -math.sin(angle), 0, 0],
                         [math.sin(angle), math.cos(angle), 0, 0],
                         [0,0,1,0],
                         [0,0,0,1]])


#take pose data with the robot
if mode == "takedata":
    import roslib
    roslib.load_manifest('WAMinterface')
    from WAMClientROSFunctions import *

    #calibration sheet origins
    sheetorigins = [[.55, 0, 0], [.85, 0, 0], [.55, .28, 0], [.85, .28, 0]]
    posespersheet = 9 #nubmer of poses per sheet

    #generate calibrationpositions (palm positions/rotations in robot base frame)
    calibrationpositions = []

    for sheetorigin in sheetorigins:
        #x away from thumb, z out of palm
        sidewaysmat = scipy.matrix([[1,0,0,0],
                                    [0,0,1,0],
                                    [0,-1,0,0],
                                    [0,0,0,1]], dtype=float)
        sidewaysorigin = sheetorigin[:]
        sidewaysorigin[2] = .077
        rotations = [0, math.pi/4, -math.pi/4]
        for rotation in rotations:
            calibrationpositions.append(zrotmat(rotation)*sidewaysmat)
            calibrationpositions[-1][0:3, 3] = scipy.matrix(sidewaysorigin).transpose()
        uprightorigin = sheetorigin[:]
        uprightorigin[1] += .107
        uprightorigin[2] = .185
        uprightmat = scipy.matrix([[-1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,-1,0],
                                   [0,0,0,1]], dtype=float)
        rotations = [0, math.pi/4, -math.pi/4, math.pi, math.pi*3/4, -math.pi*3/4]
        for rotation in rotations:
            calibrationpositions.append(zrotmat(rotation)*uprightmat)
            calibrationpositions[-1][0:3, 3] = scipy.matrix(uprightorigin).transpose()

    #print "calibrationpositions:"
    #for calibrationposition in calibrationpositions:
    #    print ppmat4tostr(calibrationposition)+"\n"


    #to be filled in
    calibrationmotorangles = [0]*len(calibrationpositions)
    barrettjointangles = [0]*len(calibrationpositions)
    barrettpositions = [0]*len(calibrationpositions)

    datafile = file("calibdata.txt", 'w')

    #toss all current data into calibdata.p
    def pickle_data():
        pickle.dump([calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions], open("calibdata.p", "w"))



    print "initializing wamik library"
    wamik.init_wamik()

    print "starting the WAM client"
    connect_WAM_client(4321)

    print "connecting to the arm"
    connect_arm()

    print "turning on gravity compensation"
    turn_on_gravity_comp()

    sheetindex = -1
    poseindex = -1

    while(1):
        print "last entered sheet:", sheetindex, "pose:", poseindex
        print "enter a sheet number to record joint angles/motor angles for a calibration location, x to pickle data, move arm home, and exit, s to just pickle current data"
        input = raw_input()
        if 'x' in input:
            break
        elif 's' in input:
            pickle_data()
            continue
        try:
            sheetindex = int(input.strip())
        except:
            print "not a valid sheet number"
            continue

        if sheetindex < 0 or sheetindex > len(sheetorigins):
            print "sheet number too high"
            continue    
        print "enter a pose number:"
        input = raw_input()
        try:
            poseindex = int(input.strip())
        except:
            print "not a valid pose number"
            continue
        if poseindex < 0 or poseindex > posespersheet:
            print "pose number too high"
            continue
        index = sheetindex*posespersheet + poseindex

        datafile.write("\nsheet"+str(sheetindex)+"pose"+str(poseindex)+"\n")
        print "getting current joint angles"
        jointangles = get_joint_angles()
        if jointangles != None:
            print "joint angles:", ppdoublelisttostr(jointangles)
            barrettjointangles[index] = jointangles
        datafile.write("barrett-reported joint angles:\n")
        datafile.write(ppdoublelisttostr(jointangles)+'\n')

        print "getting current motor angles"
        motorangles = get_motor_angles()
        print "motorangles:", ppdoublelisttostr(motorangles)    
        calibrationmotorangles[index] = motorangles
        datafile.write("motor angles:\n");
        datafile.write(ppdoublelisttostr(motorangles)+'\n')

        print "getting current supposed Cartesian position/orientation"
        (pos, rot) = get_cartesian_pos_and_rot()
        supposedrot4 = scipy.matrix(scipy.eye(4), dtype=float)
        for i in range(3):
            supposedrot4[i,3] = pos[i]
            for j in range(3):
                supposedrot4[i,j] = rot[i*3+j]
        #print ppmat4tostr(supposedrot4)

        baserot = joint0_to_base_mat*supposedrot4
        barrettpositions[index] = baserot
        print "barrett-reported rotmat relative to robot base:\n", ppmat4tostr(baserot)
        print "actual rotmat relative to robot base:\n", ppmat4tostr(calibrationpositions[index])
        print "barrett-reported position relative to robot base:", ppscipyvecttostr(baserot[0:3,3])
        print "actual position relative to robot base:          ", ppscipyvecttostr(calibrationpositions[index][0:3, 3])
        datafile.write("barrett-reported position/orientation:\n")
        datafile.write(ppmat4tostr(baserot)+'\n')
        datafile.write("actual position/orientation:\n")
        datafile.write(ppmat4tostr(calibrationpositions[index])+'\n')


    print "pickling data and sending arm home"
    pickle_data()
    arm_home()
    trajectory_wait()
    disable_controllers()

    print "closing the WAM client connection: idle the robot and press enter in the socketwamif window"
    close_WAM_client()


#compute best-fit transmission ratios
if mode == "compute_ratios":

    from scipy.optimize import *

    #starting estimate of transmission ratios 
    #(N and n from wam.conf: N[2] usually == N[1], so it's been replaced by n[2] for conciseness; likewise with N[5] and n[5])
    N = [42, 28.25, 1.68, 18, 9.4796, 1, 14.93]

    #return J2MP, the matrix to go from joint angles to motor angles
    def findJ2MP(N):
        J2MP = scipy.matrix(scipy.zeros([7,7]))
        for i in [0,3,6]:
            J2MP[i,i] = -N[i]
        J2MP[1,1] = N[1]
        J2MP[2,1] = -N[1]
        J2MP[1,2] = -N[1]/N[2]
        J2MP[2,2] = -N[1]/N[2]
        J2MP[4,4] = N[4]
        J2MP[5,4] = N[4]
        J2MP[4,5] = -N[4]/N[5]
        J2MP[5,5] = N[4]/N[5]
        return J2MP

    #return M2JP, the matrix to go from motor angles to joint angles
    def findM2JP(N):
        M2JP = scipy.matrix(scipy.zeros([7,7]))
        for i in [0,3,6]:
            M2JP[i,i] = -1./N[i]
        M2JP[1,1] = 1./N[1]/2.
        M2JP[1,2] = -M2JP[1,1]
        M2JP[2,1] = -1./(N[1]/N[2])/2.
        M2JP[2,2] = M2JP[2,1]
        M2JP[4,4] = 1./N[4]/2.
        M2JP[4,5] = M2JP[4,4]
        M2JP[5,4] = -1./(N[4]/N[5])/2.
        M2JP[5,5] = -M2JP[5,4]
        return M2JP

    #motorangles is a 7x1 scipy matrix, N is a 7-list of transmission ratios
    def convertMotorToJoint(motorangles, N):
        M2JP = findM2JP(N)
        return M2JP*motorangles

    #jointangles is a 7x1 scipy matrix, N is a 7-list of transmission ratios
    def convertJointToMotor(jointangles, N):
        J2MP = findJ2MP(N)
        return J2MP*jointangles

    [calibrationpositions, calibrationmotorangles, barrettjointangles, barrettpositions] = pickle.load(open("calibdata.p", "r"))

    #optimization value function
    def optfunc(N):
        for (motorangles, palmmat) in zip(calibrationmotorangles, calibrationpositions):
            if type(motorangles) != int:
                jointangles = convertMotorToJoint(motorangles, N)
                fkresult = 
    



