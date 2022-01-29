#!/usr/bin/env python
# ---------------------------------------------------
from multiprocessing.connection import wait
from pid import PID
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

from roblib import *  
# ---------------------------------------------------

m,g,b,d,l=10,9.81,2,1,1
I=array([[10,0,0],[0,10,0],[0,0,20]])
dt = 0.01  
B=array([[b,b,b,b],[-b*l,0,b*l,0],[0,-b*l,0,b*l],[-d,d,-d,d]])
X = array([[0.2,-0.3,0, 0,0,0]]).T
R = eye(3,3)


def control_kwad(msg, args):
    # Declare global variables as you dont want these to die, reset to zero and then re-initiate when the function is called again.
    global roll, pitch, yaw, X, R

  

    # # Convert the quaternion data to roll, pitch, yaw data
    # # The model_states contains the position, orientation, velocities of all objects in gazebo. In the simulation, there are objects like: ground, Contruction_cone, quadcopter (named as 'Kwad') etc. So 'msg.pose[ind]' will access the 'Kwad' object's pose information i.e the quadcopter's pose.
    ind = msg.name.index('Kwad')
    orientationObj = msg.pose[ind].orientation
    orientationList = [orientationObj.x, orientationObj.y,
                       orientationObj.z, orientationObj.w]
    (roll, pitch, yaw) = (euler_from_quaternion(orientationList))

    # send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
    # Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
    # (fUpdated, err_roll, err_pitch, err_yaw) = PID(roll, pitch, yaw, f)

    # The object args contains the tuple of objects (velPub, err_rollPub, err_pitchPub, err_yawPub. publish the information to namespace.
    
    vr = (X[0:3]).reshape(3,1)
    print(X.shape)
    wr = (X[3:6]).reshape(3,1)
    
    x, y, z = msg.pose[ind].position.x ,msg.pose[ind].position.y ,msg.pose[ind].position.z 
    dp = R@vr

    zd = -10
    vd  = 10
    fd = f_vdp(array([[1],[1]]))


    td0 = 300*tanh(z-zd)+ 60*vr[2]
    phid = 0.5*tanh(10*sawtooth(angle(fd)-angle(dp)))
    thetad = -0.3*tanh(vd-vr[0])
    psid = angle(dp)

    wrd = 5*inv(R)@array([[sawtooth(phid)],[sawtooth(thetad)], [sawtooth(psid)]], dtype="float64")
    td13 = I @ ((100*(wrd - wr))+ adjoint(wr)@I@wr)
    print(wrd)
    W2 = inv(B)@vstack(([td0], td13))
    
    w = sqrt(abs(W2))*sign(W2)
    

    R = eulermat(roll, pitch, yaw)
    Xdot=f(X,w, R)
    X  = X + dt*Xdot
    wr = X[3:6].reshape(3,1)

    cons = Float64MultiArray()
    cons.data =[ 0,0,0,0]
    
    args[0].publish(cons)
# ----------------------------------------------------
def f_vdp(x):
    x = x.flatten()
    vdp0 = x[1]
    vdp1 = -(0.001*(x[0]**2)-1)*x[1]-x[0]
    dx  = array([[vdp0], [vdp1]])
    return dx

def f(x,w, R):
    x=x.flatten()
    vr=(x[:3]).reshape(3,1)
    wr=(x[3:6]).reshape(3,1)
    w2=w*abs(w)
    τ=B@w2.flatten()

    dp=R@vr
    dvr=-adjoint(wr)@vr+inv(R)@array([[0],[0],[g]])+array([[0],[0],[-τ[0]/m]])
    dwr= inv(I)@(-adjoint(wr)@I@wr+τ[1:4].reshape(3,1))
    return  vstack((dvr,dwr))

def F(R,wr):
    dR = R @ adjoint(wr)
    return dR



# Initiate the node that will control the gazebo model
rospy.init_node("Control")

# initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>
err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)

# initialte publisher velPub that will publish the velocities of individual BLDC motors
velPub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=4)

# Subscribe to /gazebo/model_states to obtain the pose in quaternion form

# Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "control_kwad" function.
PoseSub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                           control_kwad, (velPub))


rospy.spin()
