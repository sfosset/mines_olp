import vrep
import math
import time
from BackProp_Python import NN


def theta(x_left, y_left, x_right, y_right):
    return math.atan2(x_right - x_left, y_left - y_right)

def distance(position, target):
    return (position[0]-target[0])**2+(position[1]-target[1])**2

def to_rad(deg):
    return 2*math.pi*deg/360

def to_deg(rad):
    return rad*360/(2*math.pi)

def theta_s(x,y):
    if x>0:
        return math.atan(10*y)
    if x<=0:
        return math.atan(-10*y)

# simulation config
ip = '127.0.0.1'
port = 19997
scene = './simu.ttt'
target = [0,0,to_rad(0)] # [x,y,theta]
position_init = [3,3,to_rad(45)]
r = 0.096 # wheel radius
R = 0.267 # demi-distance entre les r

# neural network config
ni = 3 # number of input nodes (without bias)
nj = 10 # number of hidden nodes
nk = 2 # number of output nodes

alpha = [1/4,1/4,1/(math.pi)]

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

if client_id!=-1:
    print ('Connected to remote API server on %s:%s' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    res, pioneer = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
    res, left_motor = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    res, right_motor = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    # set initial position, -1 is absolute
    vrep.simxSetObjectPosition(client_id, pioneer, -1, [position_init[0], position_init[1], 0.13879], vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectOrientation(client_id, pioneer, -1, [0, 0, to_deg(position_init[2])], vrep.simx_opmode_oneshot_wait)


    network = NN(ni,nj,nk)

    vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait)

    position = position_init # [x,y,theta]
    network_input = [0, 0, 0]
    for i in range(ni):
        network_input[i] = position[i]-target[i]

    prev_error = 100000
    #while(distance(position, target)>0.001 or position[2]-target[2]>0.1):
    while(True):
        #current_time = time.time()
        current_time = vrep.simxGetLastCmdTime(client_id)
        command = network.runNN(network_input)
        vrep.simxSetJointTargetVelocity(client_id, left_motor, 2*command[0], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(client_id, right_motor, 2*command[1], vrep.simx_opmode_oneshot_wait)

        # wait for velocity stabilization (??ms)
        time.sleep(0.050)

        # calcul prochain input
        res, tmp = vrep.simxGetObjectPosition(client_id, pioneer, -1, vrep.simx_opmode_oneshot_wait)
        position[0] = tmp[0]
        position[1] = tmp[1]

        res, tmp = vrep.simxGetObjectOrientation(client_id, pioneer, -1, vrep.simx_opmode_oneshot_wait)
        position[2] = tmp[2] # en radian



        network_input[2]=(position[2]-target[2]-theta_s(position[0], position[1]))*alpha[2]
        network_input[1] = (position[1]-target[1])*alpha[1]
        network_input[0] = (position[0]-target[0])*alpha[0]

        error = (position[0]-target[0])**2+(position[1]-target[1])**2+(position[2]-target[2])**2

        delta_t = (vrep.simxGetLastCmdTime(client_id)-current_time)/1000
        #delta_t = time.time()-current_time
        grad = [
            ((-1)/(delta_t**2))*(network_input[0]*delta_t*r*math.cos(position[2])
            +network_input[1]*delta_t*r*math.sin(position[2])
            -network_input[2]*delta_t*r/(2*R)),

            ((-1)/(delta_t**2))*(network_input[0]*delta_t*r*math.cos(position[2])
            +network_input[1]*delta_t*r*math.sin(position[2])
            +network_input[2]*delta_t*r/(2*R))
        ]

        if True:
            network.backPropagate(grad, 0.2, 0)

    # terminate
    vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(client_id)

else:
    print('Unable to connect to %s:%s' % (ip, port))
