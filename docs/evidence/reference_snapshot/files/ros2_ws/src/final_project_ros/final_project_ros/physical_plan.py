"""URDF-derived CRANE+ kinematics and the learned residual grasp parameters."""
import numpy as np
from scipy.optimize import least_squares

ARM_JOINTS=['crane_plus_joint'+str(i) for i in range(1,5)]
HAND_JOINT='crane_plus_joint_hand'
LOW=np.array([-2.61799,-1.82474,-2.55656,-1.8401])+.0001
HIGH=np.array([2.61799,1.78372,2.45428,1.76837])-.0001
BASE=np.array([.1,-.05,.05])
TRAY=np.array([.075,-.175])
OPEN=-.65

def fk(q):
    yaw,a,b,c=q
    t=a-b-c
    radial=.083*np.sin(a)+.0935*np.sin(a-b)
    position=np.array([radial*np.cos(yaw),radial*np.sin(yaw),.0714+.083*np.cos(a)+.0935*np.cos(a-b)])
    cy,sy=np.cos(yaw),np.sin(yaw);ct,st=np.cos(t),np.sin(t)
    rotation=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[ct,0,st],[0,1,0],[-st,0,ct]])
    return position,rotation

def solve(position,pitch,seed=None,offset=None):
    position=np.array(position)
    if seed is None:seed=np.array([np.arctan2(position[1],position[0]),.5,-1.5,-.8])
    def residual(q):
        p,r=fk(q)
        if offset is not None:p=p+r@offset
        return np.r_[p-position,.05*(q[1]-q[2]-q[3]-pitch)]
    result=least_squares(residual,np.clip(seed,LOW,HIGH),bounds=(LOW,HIGH),max_nfev=200)
    if np.linalg.norm(residual(result.x))>.004:
        raise ValueError('No reachable solution for physical delivery waypoint: '+str(position))
    return result.x

def grasp_plan(object_relative,parameters):
    dr,dy,height,pitch,close,move_s,close_s,lift_s=parameters
    xy=np.array(object_relative[:2]);yaw=np.arctan2(xy[1],xy[0])
    rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    wrist=np.r_[xy+rot@[dr,dy],height+object_relative[2]-.02]
    pick=solve(wrist,pitch)
    lift_pitch=pitch-.65
    # Match the training controller's wrist reorientation.
    def rotation(angle):
        c,s=np.cos(angle),np.sin(angle)
        return np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])@np.array([[c,0,s],[0,1,0],[-s,0,c]])
    lift_wrist=wrist+(rotation(pitch)-rotation(lift_pitch))@np.array([0.,.015,.11])+[0,0,.065]
    lift=solve(lift_wrist,lift_pitch)
    return dict(pick=pick,lift=lift,close=close,move_s=move_s,close_s=close_s,lift_s=lift_s,pitch=pitch)

def box_bounds(position,quaternion,half_size=.02):
    """World AABB of all eight box corners; quaternion order is x,y,z,w."""
    x,y,z,w=np.array(quaternion)/np.linalg.norm(quaternion)
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    corners=np.array([[a,b,c] for a in [-half_size,half_size] for b in [-half_size,half_size] for c in [-half_size,half_size]])@r.T+position
    return corners.min(axis=0),corners.max(axis=0)

def placement_confirmed(position,quaternion,speed,bilateral_contact,floor_contact):
    lower,upper=box_bounds(position,quaternion)
    return bool(np.all(lower[:2]>=TRAY-.07-.001) and np.all(upper[:2]<=TRAY+.07+.001)
                and .057<lower[2]<.063 and upper[2]<.115
                and speed<.02 and not bilateral_contact and floor_contact)


def gripper_bottom(q,hand,fixed_vertices,hand_vertices):
    """Lowest world-space point of the actual finger collision meshes."""
    fixed=fixed_vertices@np.array([[1,0,0],[0,0,-1],[0,1,0]]).T
    moving=hand_vertices@np.array([[0,0,-1],[0,1,0],[1,0,0]]).T
    c,s=np.cos(hand),np.sin(hand)
    moving=moving@np.array([[1,0,0],[0,c,s],[0,-s,c]]).T+[0,-.0145,.045]
    p,r=fk(q)
    points=np.vstack([fixed,moving])@r.T+p+BASE
    return float(points[:,2].min())
