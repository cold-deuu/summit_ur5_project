import rospy
from sensor_msgs.msg import JointState
from scipy.io import loadmat
import numpy as np
import rospkg

q = np.array([])

def tanh(x):
    return np.tanh(x)

def softmax(x):
    e_x = np.exp(x - np.max(x)) 
    return e_x / e_x.sum(axis=0)

def forward_pass(x, weights):
    x = x.reshape(-1, 1) 

    print(f"Bias : {weights['l1_bias'].shape}")
    print(f"Weight : {weights['l1_weight'].shape}")
    
    print(f"Bias : {weights['l2_bias'].shape}")
    print(f"Weight : {weights['l2_weight'].shape}")
    print(f"Bias : {weights['l3_bias'].shape}")
    print(f"Weight : {weights['l3_weight'].shape}")
    print(f"Bias : {weights['l4_bias'].shape}")
    print(f"Weight : {weights['l4_weight'].shape}")
    print(f"Bias : {weights['l5_bias'].shape}")
    print(f"Weight : {weights['l5_weight'].shape}")
    l1 = tanh(weights['l1_weight'] @ x + weights['l1_bias'].reshape(-1, 1))
    l2 = tanh(weights['l2_weight'] @ l1 + weights['l2_bias'].reshape(-1, 1))
    l3 = tanh(weights['l3_weight'] @ l2 + weights['l3_bias'].reshape(-1, 1))
    l4 = tanh(weights['l4_weight'] @ l3 + weights['l4_bias'].reshape(-1, 1))
    out = weights['l5_weight'] @ l4+ weights['l5_bias'].reshape(-1, 1)
    out = softmax(out) 
    return out.flatten()  




def joint_state_callback(msg):
    global q
    q = np.array(msg.position)  # 최신 joint 상태를 numpy 배열로 저장

def main():
    global q
    rospy.init_node('joint_state_listener', anonymous=True)
    
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    
    rate = rospy.Rate(10)  # 10 Hz
    

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sca_pkg')
    mat = loadmat(package_path + '/src/nn_mm_100.mat')     
    weights = {
        'l1_weight': mat['l1_weight'],
        'l1_bias': mat['l1_bias'].flatten(),
        'l2_weight': mat['l2_weight'],
        'l2_bias': mat['l2_bias'].flatten(),
        'l3_weight': mat['l3_weight'],
        'l3_bias': mat['l3_bias'].flatten(),
        'l4_weight': mat['l4_weight'],
        'l4_bias': mat['l4_bias'].flatten(),
        'l5_weight': mat['l5_weight'],
        'l5_bias': mat['l5_bias'].flatten(),
    }



    while not rospy.is_shutdown():
        if q.size > 0:
            # forward 계산
            y = forward_pass(q, weights)
            max_index = np.argmax(y) 
            # print(y)
            if y[1] > 0.5 and y[1]<0.9:
                rospy.logwarn("Near Collision")
            elif y[1]>0.9:
                rospy.logerr("Collision")
            else:
                rospy.loginfo("Stable")

            # if(max_index==0):
            #     print("Stable")
            # else:
            #     print("Self-Collision")
        else:
            rospy.loginfo("Waiting for joint_states...")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
