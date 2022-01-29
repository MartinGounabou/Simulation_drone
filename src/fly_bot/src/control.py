#!/usr/bin/env python
# ---------------------------------------------------
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
X = array([[50,-50,0, 0,0,0]]).T
R = eye(3,3)

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

    
    vr = (X[0:3]).reshape(3,1)
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
    W2 = inv(B)@vstack(([td0], td13))
    
    w = sqrt(abs(W2))*sign(W2)
    

    R = eulermat(roll, pitch, yaw)
    Xdot=f(X,w, R)
    X  = X + dt*Xdot
    wr = X[3:6].reshape(3,1)




    w = w.flatten()
    w[1] = -sign(w[0])* abs(w[1])
    w[3] = -sign(w[2])* abs(w[3])
    
    esc_br = 1500 + w[3]

    esc_bl = 1500 + w[2]

    esc_fl = 1500 - w[1]

    esc_fr = 1500 - w[0]


    if(esc_br > 2000):
        esc_br = 2000
    if(esc_bl > 2000):
        esc_bl = 2000
    if(esc_fr > 2000):
        esc_fr = 2000
    if(esc_fl > 2000):
        esc_fl = 2000

    if(esc_br < 1100):
        esc_br = 1100
    if(esc_bl < 1100):
        esc_bl = 1100
    if(esc_fr < 1100):
        esc_fr = 1100
    if(esc_fl < 1100):
        esc_fl = 1100

    # Map the esc values to motor values
    br_motor_vel = ((esc_br - 1500)/25) + 50
    bl_motor_vel = ((esc_bl - 1500)/25) + 50
    fr_motor_vel = ((esc_fr - 1500)/25) + 50
    fl_motor_vel = ((esc_fl - 1500)/25) + 50
    
    cons = Float64MultiArray()
    cons.data = [fr_motor_vel, -fl_motor_vel, bl_motor_vel, -br_motor_vel]
    
    args[0].publish(cons)
# ------------------------
# ----------------------------------------------------



# Initiate the node that will control the gazebo model
rospy.init_node("Control")

# initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>
err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)


velPub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=4)


PoseSub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                           control_kwad, (velPub, err_rollPub, err_pitchPub, err_yawPub))


rospy.spin()