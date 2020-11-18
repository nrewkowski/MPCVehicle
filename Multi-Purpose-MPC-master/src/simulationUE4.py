from map import Map, Obstacle
import numpy as np
from reference_path import ReferencePath
from spatial_bicycle_models import BicycleModel
import matplotlib.pyplot as plt
from MPC import MPC
from scipy import sparse
import socket

returnUDPMessage=""
mpc=None
car=None
x_log=None
y_log=None
v_log=None
t=None
reference_path=None
sock=None
receivedReply=True
map=None
def setupMPC():
    global returnUDPMessage
    global mpc
    global car
    global x_log
    global y_log
    global v_log
    global t
    global reference_path
    global map
    print("go")
    # Select Simulation Mode | 'Sim_Track' or 'Real_Track'
    sim_mode = 'Sim_Track'

    # Simulation Environment. Mini-Car on track specifically designed to show-
    # case time-optimal driving.
    if sim_mode == 'Sim_Track':

        # Load map file
        map = Map(file_path='maps/sim_map5.png', origin=[-1, -2],
                  resolution=0.005) #picture is 35x35m 

                  #500x500px

        """
        Constructor for map object. Map contains occupancy grid map data of
        environment as well as meta information.
        :param file_path: path to image of map
        :param threshold_occupied: threshold value for binarization of map
        image
        :param origin: x and y coordinates of map origin in world coordinates
        [m]
        :param resolution: resolution in m/px
        """

        # Specify waypoints
        wp_x = [-0.75,  -0.25,-0.25,  0.25,  0.25, 1.25, 1.25, 0.75, 0.75, 1.25, 1.25, -0.75, -0.75, -0.25]
        wp_y = [-1.5,   -1.5, -0.5,  -0.5,  -1.5, -1.5, -1,   -1,   -0.5,  -0.5, 0,     0,    -1.5,  -1.5]

        # Specify path resolution
        path_resolution = 0.05  # m / wp

        # Create smoothed reference path
        reference_path = ReferencePath(map, wp_x, wp_y, path_resolution,
                                       smoothing_distance=5, max_width=0.23,
                                       circular=True)

        # Add obstacles
        use_obstacles = False
        if use_obstacles:
            obs1 = Obstacle(cx=0.0, cy=0.0, radius=0.05)
            obs2 = Obstacle(cx=-0.8, cy=-0.5, radius=0.08)
            obs3 = Obstacle(cx=-0.7, cy=-1.5, radius=0.05)
            obs4 = Obstacle(cx=-0.3, cy=-1.0, radius=0.08)
            obs5 = Obstacle(cx=0.27, cy=-1.0, radius=0.05)
            obs6 = Obstacle(cx=0.78, cy=-1.47, radius=0.05)
            obs7 = Obstacle(cx=0.73, cy=-0.9, radius=0.07)
            obs8 = Obstacle(cx=1.2, cy=0.0, radius=0.08)
            obs9 = Obstacle(cx=0.67, cy=-0.05, radius=0.06)
            map.add_obstacles([obs1, obs2, obs3, obs4, obs5, obs6, obs7,
                                          obs8, obs9])
        #map.remove_obstacle(obs4)
        """
        Simplified Spatial Bicycle Model. Spatial Reformulation of Kinematic
        Bicycle Model. Uses Simplified Spatial State.
        :param reference_path: reference path model is supposed to follow
        :param length: length of the car in m
        :param width: with of the car in m
        :param Ts: sampling time of model in s
        """
        # Instantiate motion model
        car = BicycleModel(length=0.12, width=0.06, #12cm. 6cm
                           reference_path=reference_path, Ts=0.008333333333)

    else:
        print('Invalid Simulation Mode!')
        map, wp_x, wp_y, path_resolution, reference_path, car \
            = None, None, None, None, None, None
        exit(1)

    ##############
    # Controller #
    ##############
        """
        Constructor for the Model Predictive Controller.
        :param model: bicycle model object to be controlled
        :param N: time horizon | int
        :param Q: state cost matrix
        :param R: input cost matrix
        :param QN: final state cost matrix
        :param StateConstraints: dictionary of state constraints
        :param InputConstraints: dictionary of input constraints
        :param ay_max: maximum allowed lateral acceleration in curves
        """
    N = 30
    Q = sparse.diags([1.0, 0.0, 0.0])
    R = sparse.diags([0.5, 0.0])
    QN = sparse.diags([1.0, 0.0, 0.0])

    v_max = 1  # m/s
    delta_max = .66  # rad
    ay_max = 4000  # m/s^2 #max laterial accel (how far it's actually capable of going)
    InputConstraints = {'umin': np.array([0.0, -np.tan(delta_max)/car.length]),
                        'umax': np.array([v_max, np.tan(delta_max)/car.length])}
    StateConstraints = {'xmin': np.array([-np.inf, -np.inf, -np.inf]),
                        'xmax': np.array([np.inf, np.inf, np.inf])}
    mpc = MPC(car, N, Q, R, QN, StateConstraints, InputConstraints, ay_max)

    # Compute speed profile
    a_min = -0.1  # m/s^2
    a_max = 0.5  # m/s^2   ##################what inputs the car can actually accept

    """
        Compute a speed profile for the path. Assign a reference velocity
        to each waypoint based on its curvature.
        :param Constraints: constraints on acceleration and velocity
        curvature of the path
        """
    SpeedProfileConstraints = {'a_min': a_min, 'a_max': a_max,
                               'v_min': 0.0, 'v_max': v_max, 'ay_max': ay_max}
    car.reference_path.compute_speed_profile(SpeedProfileConstraints)

    ##############
    # Simulation #
    ##############

    # Set simulation time to zero
    t = 0.0

    # Logging containers
    x_log = [car.temporal_state.x]
    y_log = [car.temporal_state.y]
    v_log = [0.0]
    returnUDPMessage="DONE SETUP"
    print(returnUDPMessage)

def doMPC():
    global receivedReply
    global returnUDPMessage
    global mpc
    global car
    global x_log
    global y_log
    global v_log
    global t
    global reference_path
    global sock
    # Until arrival at end of path
    #while car.s < reference_path.length:

        # Get control signals
    u = mpc.get_control()
    #print(u)

    #sockrcv.sendto(("DRIVE,"+str(u[0])+","+str(u[1])).encode(), (UDP_IP, sendPort))
    #receivedReply=False
    #print("SENT","DRIVE,"+str(u[0])+","+str(u[1]))
    #make ue4 do it
    #while not receivedReply:
        #data, addr = sock.recvfrom(1024)
        #print("received message: %s" % data)
        #receivedReply=True
    #replyInfo=data.decode("utf-8").split(',')
    #print("ue4 replied")
    # Simulate car
    car.drive(u)
    #wait for UE4 to reply
    sockrcv.sendto(("DRIVE,"+str(car.temporal_state.x)+","+str(car.temporal_state.y)+","+str(car.temporal_state.psi)).encode(), (UDP_IP, sendPort))
    # Log car state
    # x_log.append(float(replyInfo[1]))
    # y_log.append(float(replyInfo[2]))
    # v_log.append(u[0])

    x_log.append(car.temporal_state.x)
    y_log.append(car.temporal_state.y)
    v_log.append(u[0])

    # Increment simulation time
    #print("carts",car.Ts)
    t += car.Ts

    # Increment simulation time
    #t += float(replyInfo[0]*10)
    #print("T IS",t)

    # Plot path and drivable area
    #reference_path.show()

    # Plot car
    #car.show()

    # Plot MPC prediction
    #mpc.show_prediction()

    # Set figure title
    #plt.title('MPC Simulation: v(t): {:.2f}, delta(t): {:.2f}, Duration: '
                #'{:.2f} s'.format(u[0], u[1], t))
    #plt.axis('off')
    #plt.pause(0.001)

def takeUDPCOmmand(command):
    global mpc
    global car
    global x_log
    global y_log
    global v_log
    global t
    global reference_path
    global map
    global returnUDPMessage
    commandSplit=command.split(",")
    if commandSplit[0]=="setup":
        setupMPC()
    elif commandSplit[0]=="quit":
        print("QUIT!!!")
        quit()
    elif commandSplit[0]=="next":
        doMPC()
    elif commandSplit[0]=="addObstacle":
        newObs = Obstacle(cx=float(commandSplit[1]), cy=float(commandSplit[2]), radius=float(commandSplit[3]))
        map.add_obstacles([newObs])
    else:
        print("unknown command:",commandSplit)
    #pass

#def returnUDPMessage
if __name__ == '__main__':
    #global receivedReply
    print("main")
    UDP_IP = "192.168.43.21"
    UDP_PORT = 8550
    sendPort = 8551
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    sockrcv = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    setupMPC()
    while True:
        #global returnUDPMessage
        if receivedReply:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            print("received message: %s" % data)
            takeUDPCOmmand(data.decode("utf-8") )
            if returnUDPMessage != "":
                sockrcv.sendto(returnUDPMessage.encode(), (UDP_IP, sendPort))
                returnUDPMessage=""
        else:
            pass
    #MESSAGE = b"Hello, World!"
    #setupMPC()
    #doMPC()