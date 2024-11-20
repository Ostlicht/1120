#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R_
import shutil

from beginner_tutorials.srv import vision_vertices


from beginner_tutorials.srv import servoing, servoingRequest
from beginner_tutorials.srv import servoing_posx, servoing_posxResponse
from beginner_tutorials.srv import findPickposx, findPickposxResponse
from get_obj_info.srv import obj_info, obj_infoRequest


from beginner_tutorials.msg import servoing_plot
from beginner_tutorials.msg import pick_plot

from dsr_msgs.srv import MoveJointx, MoveJointxRequest
from dsr_msgs.srv import MoveJoint, MoveJointRequest
from dsr_msgs.srv import MoveLine, MoveLineRequest
from dsr_msgs.srv import GetCurrentPose, GetCurrentPoseRequest

################그리퍼 조절 함수 ######################################
import time
import rospy
from dsr_msgs.srv import SetCtrlBoxDigitalOutput
from dsr_msgs.srv import SetCtrlBoxDigitalOutput, SetCtrlBoxDigitalOutputRequest

def set_digital_output(nGpioIndex, bGpioValue):
    
    try:
        # Create a service proxy
        srvSetCtrlBoxDigitalOutput = rospy.ServiceProxy('/dsr01m1013/io/set_digital_output', SetCtrlBoxDigitalOutput)
        
        # Create the service request
        srv = SetCtrlBoxDigitalOutputRequest()
        srv.index = nGpioIndex
        srv.value = bGpioValue
        
        # Call the service
        response = srvSetCtrlBoxDigitalOutput(srv)
        
        if response.success:
            return response.success
        else:
            rospy.logerr("Failed to set digital output")
            return -1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        rospy.signal_shutdown("ROS shutdown due to service failure.")
        return -1

def move_gripper(state, distance_mm):
    # 스트로크가 10mm이고, 전체 열림/닫힘 시간이 0.5초일 때
    full_stroke = 10.0  # 전체 스트로크 10mm
    full_time = 0.5  # 열리고 닫히는 시간 0.5초

    # 움직일 비율 (5mm는 전체 스트로크의 절반이므로 0.5)
    ratio = distance_mm / full_stroke
    move_time = full_time * ratio  # 움직이는 데 걸릴 시간 계산

    # state 0 -> off, state 1 -> on, else -> stop
    if state == 0:  # 그리퍼 열기
        set_digital_output(1, True)  # 그리퍼 열기 시작
        set_digital_output(2, False)
        time.sleep(move_time)  # 5mm에 해당하는 시간만큼 대기
        set_digital_output(1, False)  # 그리퍼 멈춤
        set_digital_output(2, False)
    elif state == 1:  # 그리퍼 닫기
        set_digital_output(1, False)  # 그리퍼 닫기 시작
        set_digital_output(2, True)
        time.sleep(move_time)  # 5mm에 해당하는 시간만큼 대기
        set_digital_output(1, False)  # 그리퍼 멈춤
        set_digital_output(2, False)
    else:  # 그리퍼 멈추기
        set_digital_output(1, False)
        set_digital_output(2, False)
################그리퍼 조절 함수 ######################################
        

# -------------------------------------------------------두산로봇 함수 ----------------------------------------------------------------------------------------

def movejx(posx, vel, acc, time, object_x=0, object_y=0, object_z=0, sol=2):


    # 서비스 클라이언트 생성
    rospy.wait_for_service('/dsr01m1013/motion/move_jointx')
    try:
        move_jointx = rospy.ServiceProxy('/dsr01m1013/motion/move_jointx', MoveJointx)
        prepose = np.array(get_current_pose(0))    


        # posxG 계산
        posxg = posx_calculate(posx, object_x, object_y, object_z - 230)

        # 서비스 요청 데이터 설정
        req = MoveJointxRequest()
        req.pos = posxg
        req.vel = vel
        req.acc = acc
        req.sol = sol
        req.time = time
        req.radius = 0
        req.mode = 0
        req.blendType = 0
        req.syncType = 0

        response = move_jointx(req)

        if response:
            current_pose = np.array(get_current_pose(0))
            posdiff = current_pose - prepose
            if(np.linalg.norm(posdiff) < 0.1):
                return False
            else :
                return True
        else:
            rospy.logerr("Failed to call service move_joint")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def movej(pos, vel, acc, time):
    # ROS 노드 초기화

    # 서비스 클라이언트 생성
    rospy.wait_for_service('/dsr01m1013/motion/move_joint')
    try:
        move_joint_service = rospy.ServiceProxy('/dsr01m1013/motion/move_joint', MoveJoint)

        # 서비스 요청 생성
        srv = MoveJointRequest()
        srv.pos = pos
        srv.vel = vel
        srv.acc = acc
        srv.time = time
        srv.radius = 0
        srv.mode = 0
        srv.blendType = 0
        srv.syncType = 0

        # 서비스 호출
        response = move_joint_service(srv)
        # if response:
        #     rospy.loginfo("Success")
        # else:
        #     rospy.logerr("Failed to call service move_joint")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def moveline(posx, vel, acc, time):
    rospy.wait_for_service('/dsr01m1013/motion/move_line')
    try:
        move_line = rospy.ServiceProxy('/dsr01m1013/motion/move_line', MoveLine)
        req = MoveLineRequest()
        req.pos = posx
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = 0
        req.mode = 0
        req.blendType = 0
        req.syncType = 0

        response = move_line(req)
        
        # rospy.loginfo("MoveLine Success: %f %f %f %f %f %f", *posx)
    except rospy.ServiceException as e:
        #rospy.logerr("Failed to call service move_line: %s", e)
        rospy.signal_shutdown("Service call failed")


def get_current_pose(space_type): # 0 : joint 1: space
    # ROS 노드가 초기화되어 있는지 확인
    # if not rospy.core.is_initialized():
    #     rospy.init_node('get_current_pose_client', anonymous=True)
    
    # 서비스 클라이언트 생성
    rospy.wait_for_service('/dsr01m1013/system/get_current_pose')
    try:

        get_current_pose = rospy.ServiceProxy('/dsr01m1013/system/get_current_pose', GetCurrentPose)
        req = GetCurrentPoseRequest()
        req.space_type = space_type

        response = get_current_pose(req)

        current_pose = response.pos

        return current_pose  # 위치 값 리스트를 반환

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None






# ---------------------------------------------------------------------------------------------------------------------------------------------------------

# ----------------------------------------------------다른 클라이언트 관련 함수-----------------------------------------------------------------------------------------

def servoing_func(rough_vertices):
    """
    Calls the '/collision_detect/servoing' service to refine rough vertices to precise vertices.
    Retries until the service call succeeds.
    """
    rospy.wait_for_service('/collision_detect/servoing')

    while not rospy.is_shutdown():
        try:
            # Create the service client
            servoing_client = rospy.ServiceProxy('/collision_detect/servoing', servoing)

            # Create and populate the request object
            req = servoingRequest()
            req.rough_vertices = rough_vertices.reshape(-1)

            # Call the service
            response = servoing_client(req)

            # Process the response
            correct_vertices = response.correct_vertices
            np_current_vertices = np.array(correct_vertices).reshape(8, 3)

            return np_current_vertices  # Return the refined vertices

        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call to '/collision_detect/servoing' failed: {e}")
            rospy.sleep(1)  # Wait and retry

    return None  # Return None if the ROS node shuts down


def packing(xyz, name):
    """
    Calls the '/get_obj_info_service' service to get packing information.
    Retries until the service call succeeds.
    """
    rospy.wait_for_service('/get_obj_info_service')

    while not rospy.is_shutdown():
        try:
            # Create the service client
            packing_client = rospy.ServiceProxy('/get_obj_info_service', obj_info)

            # Create and populate the request object
            req = obj_infoRequest()
            
            
            #########################
            
            if name == "color_logo" :
                req.x = ['box', 'white']
            elif name == "black_logo" :
                req.x = ['box', 'black']
            elif name == "round_logo" :
                req.x = ['box', 'orange']
            elif name == "blue_robot" :
                req.x = ['box', 'blue']
            elif name == "red_robot" :
                req.x = ['box', 'red']
            
            
            
            
            
            #########################
            
            # req.x = ['box', 'red']  # Example category and attribute
            req.y = np.rint(xyz).astype(int).reshape(-1)  # Convert to rounded integers and list
            print("req.y = ", req.y)

            # Call the service
            response = packing_client(req)

            # Process the response
            packing_xyz = np.array([response.width, response.height, response.depth])
            packing_center_position = np.array([response.x, response.y, response.z])

            return packing_xyz, packing_center_position  # Return packing dimensions and center

        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call to '/get_obj_info_service' failed: {e}")
            rospy.sleep(1)  # Wait and retry

    return None, None  # Return None if the ROS node shuts down



# ---------------------------------------------------------------------------------------------------------------------------------------------------------
def plot_box(ax, vertices, color='cyan'):
    # Define the six faces of the box using the vertices
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]], # Bottom face
        [vertices[4], vertices[5], vertices[6], vertices[7]], # Top face
        [vertices[0], vertices[1], vertices[5], vertices[4]], # Front face
        [vertices[2], vertices[3], vertices[7], vertices[6]], # Back face
        [vertices[1], vertices[2], vertices[6], vertices[5]], # Right face
        [vertices[4], vertices[7], vertices[3], vertices[0]]  # Left face
    ]
    
    # Create a 3D polygon collection and add it to the plot
    box = Poly3DCollection(faces, linewidths=1, edgecolors='r')
    box.set_facecolor(color)
    ax.add_collection3d(box)

def plot_plane(ax, p1, p2, p3, p4):
    # 주어진 점들을 NumPy 배열로 정의합니다.
    p1, p2, p3, p4 = map(np.array, [p1, p2, p3, p4])
    
    # 네 점을 이은 다각형 평면을 정의합니다.
    verts = [[p1, p2, p3, p4]]
    poly = Poly3DCollection(verts, color="cyan", alpha=0.5)
    ax.add_collection3d(poly)

    # 네 점 플롯
    # ax.scatter(*p1, color="red", s=50, label="Point 1")
    # ax.scatter(*p2, color="green", s=50, label="Point 2")
    # ax.scatter(*p3, color="blue", s=50, label="Point 3")
    # ax.scatter(*p4, color="purple", s=50, label="Point 4")
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

def plot_line(ax, p1, p2, label=None, color = 'black'):
    # 선분을 그리는 함수 (선의 굵기를 3으로 설정)
    line_x = [p1[0], p2[0]]
    line_y = [p1[1], p2[1]]
    line_z = [p1[2], p2[2]]
    
    # label이 있으면 적용, 없으면 label 매개변수를 생략
    if label is not None:
        ax.plot(line_x, line_y, line_z, color = color, linewidth=3, label=label)
        ax.legend()
    else:
        ax.plot(line_x, line_y, line_z, color = color, linewidth=3)  # label 없이 선만 그리기

def rotate_point_around_axis(p1, p2, point, theta):
    """
    주어진 축(p1, p2)과 각도(theta)만큼 임의의 점(point)을 회전시킵니다.
    
    Parameters:
    - p1, p2: 회전 축의 두 점 (numpy array 또는 list)
    - point: 회전시킬 점 (numpy array 또는 list)
    - theta: 회전 각도 (라디안)
    
    Returns:
    - rotated_point: 회전된 점의 좌표 (numpy array)
    """
    # p1, p2, point를 numpy 배열로 변환
    p1 = np.array(p1)
    p2 = np.array(p2)
    point = np.array(point)
    
    # 회전축 벡터 정의 (단위 벡터로 정규화)
    axis = p2 - p1
    axis = axis / np.linalg.norm(axis)
    
    # 회전 변환 정의
    rotation = R_.from_rotvec(np.array(theta * axis))
    
    # 회전 적용 (축의 기준점을 원점으로 이동한 후 회전하고 다시 원래 위치로 이동)
    rotated_point = rotation.apply(point - p1) + p1
    
    return rotated_point

def get_rpy(R): # R = np.array([xvec,yvec,zvec]).T
    if R[2, 2] == 1.0:
        beta = np.pi
        alpha = 0
        gamma = np.arctan2(R[0, 1], R[0, 0])
    elif R[2, 2] == -1.0:
        beta = np.pi
        alpha = 0
        gamma = np.arctan2(-R[0, 1], R[0, 0])
    else:
        beta = np.arccos(R[2, 2])
        sin_beta = np.sin(beta)
        alpha = np.arctan2(R[1, 2] / sin_beta, R[0, 2] / sin_beta)
        gamma = np.arctan2(R[2, 1] / sin_beta, -R[2, 0] / sin_beta)
    return np.degrees(alpha), np.degrees(beta), np.degrees(gamma)



# DEG to RAD 변환 함수
def DEG2RAD(deg):
    return deg * np.pi / 180.0

# 3x3 행렬과 3x1 벡터 곱셈
def matVecMult(mat, vec):
    return np.dot(mat, vec)

# Z축 회전 행렬
def rotZ(angle):
    rad = DEG2RAD(angle)
    return np.array([
        [np.cos(rad), -np.sin(rad), 0.0],
        [np.sin(rad),  np.cos(rad), 0.0],
        [0.0, 0.0, 1.0]
    ])

# Y축 회전 행렬
def rotY(angle):
    rad = DEG2RAD(angle)
    return np.array([
        [np.cos(rad), 0.0, np.sin(rad)],
        [0.0, 1.0, 0.0],
        [-np.sin(rad), 0.0, np.cos(rad)]
    ])

# ZYZ 회전 행렬을 얻는 함수
def getZYZRotationMatrix(yawZ1, pitchY, rollZ2):
    Rz1 = rotZ(yawZ1)
    Ry = rotY(pitchY)
    Rz2 = rotZ(rollZ2)

    # Rz1 * Ry * Rz2 행렬 곱
    return np.dot(np.dot(Rz1, Ry), Rz2)

# RPY 값을 받아 회전 벡터 반환
def rpy2vector(yawZ1, pitchY, rollZ2):
    rotationMatrix = getZYZRotationMatrix(yawZ1, pitchY, rollZ2)
    unitVectors = np.identity(3)  # 단위 벡터 [1, 0, 0], [0, 1, 0], [0, 0, 1]

    resultVec = np.dot(rotationMatrix, unitVectors.T).T  # 각 벡터에 회전 적용
    return resultVec

# 평행이동된 posx를 받아 평행이동 전 posx(moveline의 명령) 구함
def posx_calculate(posx, object_x, object_y, object_z):
    temp_posx = posx.copy()
    unitVec = rpy2vector(posx[3],posx[4],posx[5])

    forward = np.array([object_x, object_y, object_z])
    
    for i in range(3):
        for j in range(3):
            temp_posx[j] = temp_posx[j] +unitVec[i][j] * forward[i]

    return temp_posx

# 평행이동 된 posx와 그리퍼의 체적(상태)을 받아 그리퍼의 체적을 선으로 계산 volume은 np.array(--> 기준점 평행이동[x,y,z].[x,y,z]<-- 체적) 모두 양수로 계산
def posx2line(posx, volume):
    # 먼저 평행이동된 점 posx_p를 구하자
    unitVec = rpy2vector(posx[3],posx[4],posx[5])
    
    posx_p = posx_calculate(posx, volume[0][0], volume[0][1], volume[0][2])
    posx_xyz = np.array([posx_p[0], posx_p[1], posx_p[2]])

    

    # gripper_volume 은 ([[x+,y+,z+],[x+,y+,z-], ...]) 1분면부터 4분면 까지
    # gripper_vector 는 ([xx, xy, xz],[yx, yy, yz],[zx,zy,zz])
    gripper_vector = np.array([unitVec[0] * volume[1][0],unitVec[1] * volume[1][1],unitVec[2] * volume[1][2]])

    gripper_volume = np.array([[posx_xyz + gripper_vector[0] + gripper_vector[1] + gripper_vector[2],
                                posx_xyz + gripper_vector[0] + gripper_vector[1]],    #1사분면 라인

                                [posx_xyz - gripper_vector[0] + gripper_vector[1] + gripper_vector[2],
                                posx_xyz - gripper_vector[0] + gripper_vector[1]],    #2사분면 라인
                                
                                [posx_xyz - gripper_vector[0] - gripper_vector[1] + gripper_vector[2],
                                posx_xyz - gripper_vector[0] - gripper_vector[1]],    #3사분면 라인

                                [posx_xyz + gripper_vector[0] - gripper_vector[1] + gripper_vector[2],
                                posx_xyz + gripper_vector[0] - gripper_vector[1]]])   #4사분면 라인

    return gripper_volume






def line_pierce_surface(ax,surface, line):
    # line이 surface를 관통하는지 검사하는 함수
    # input type
    # surface =  np array 4개의 점 [[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],[x4,y4,z4]] 대신 점들은 반시계 방향, 시계 방향 둘중 하나로 일관성 있는 순서로 들어온다.
    # line은 [[x1,y1,z1],[x2,y2,z2]] 이다.
    # 결과는 안 부딪힐시엔 False, 부딪힐시엔 True와 +z축 방향으로 line을 얼마나 회전 이동 시켜야 하는지를 리턴한다. (끝점은 같아야함)

    # surface에서 임의의 세개의 점들을 뽑아 외적을 통해 normal 벡터를 구한다 크기는 규격화 x   + surface를 이루는 모든 line을 정의한다

    v1 = surface[0]-surface[1]
    v2 = surface[1]-surface[2]
    v3 = surface[2]-surface[3]
    v4 = surface[3]-surface[0]

    normal_vec = np.cross(v1,v2) #[x1,y1,z1] 평면 방정식은 normal_vec[0]x + normal_vec[1]y + normal_vec[2]z + d -= 0;
    line_vector = line[0]-line[1]

    d = -np.dot(normal_vec,surface[0])


    # 직선과 surface가 만나는 점을 계산한다

    t = -(np.dot(normal_vec,line[0])+d)/(np.dot(line_vector, normal_vec))

    x = line_vector[0]*t + line[0][0]
    y = line_vector[1]*t + line[0][1]
    z = line_vector[2]*t + line[0][2]

    point = np.array([x,y,z]) # point: 직선이 surface와 만나는 점


    space_in = 'out'

    if(pil(line, point)):

        line_surface_vector= np.array([[]])

        for i in range(surface.shape[0]):
            line_surface_vector = np.append(line_surface_vector, point - surface[i])     # point가 끝점, surface가 시작점인 벡터 4개
        line_surface_vector = line_surface_vector.reshape(4,3)

        #line_surface_vector 끼리 곱셈 만약 - 발견시 in 이라고 판단 and break

        found_negative_dot_product = False

        for i in range(line_surface_vector.shape[0]):
            for j in range(line_surface_vector.shape[0]):
                dot_product = np.dot(line_surface_vector[i], line_surface_vector[j])
                if (dot_product < 0):
                    space_in = 'in'
                    ax.scatter(point[0], point[1], point[2], color="orange", s=20, label="point")
                    found_negative_dot_product = True
                    break
            
            if found_negative_dot_product:
                break  # 외부 루프 종료


















def project_surface2line(surface, line):
    # 평면에 선을 투과 시키는 함수
    # input type은 둘다 np array로 surface는 4개의 점 [[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],[x4,y4,z4]] 대신 점들은 반시계 방향, 시계 방향 둘중 하나로 일관성 있는 순서로 들어온다.
    # line은 [[x1,y1,z1],[x2,y2,z2]] 이다.
    # 결과는 안 부딪힐시엔 False, 부딪힐시엔 True와 +z축 방향으로 line을 얼마나 회전해야 하는지를 return 한다.




    #surface에서 임의의 세개의 점들을 뽑아 외적을 통해 normal 벡터를 구한다 크기는 규격화 x   + surface를 이루는 모든 line을 정의한다

    v1 = surface[0]-surface[1]
    v2 = surface[1]-surface[2]
    v3 = surface[2]-surface[3]
    v4 = surface[3]-surface[0]

    line_vector = line[0]-line[1]


    normal_vec = np.cross(v1,v2) #[x1,y1,z1] 평면 방정식은 normal_vec[0]x + normal_vec[1]y + normal_vec[2]z + d -= 0;
    d = -np.dot(normal_vec,surface[0])

    # 방향은 normal_vec이고 line의 선을 지나는 직선을 찾아야 한다. (x-line[0][0]...)/normal_vec[0] = (y-line[1])/normal_vec[1] = (z-line[2])/normal_vec[2]
    # normal_vec[0,1 2]가 0에 가까우면 x, y, z = x1, y1, z1 매개변수로 해결하면 t = sqrt(  (-a*line[0]-b*line[1]-c*line[2]-d)/(a+b+c)   ) abc 는 normal_vec

    contact = np.array([[]])

    for i in range(2):
        t= np.sqrt((-normal_vec[0]*line[i][0]-normal_vec[1]*line[i][1]-normal_vec[2]*line[i][2]-d)/(normal_vec[0]+normal_vec[1]+normal_vec[2]))
        # 접점 contact_x1, contact_y1, contact_z1은

        contact_x = normal_vec[0]*t + line[i][0]
        contact_y = normal_vec[1]*t + line[i][1]
        contact_z = normal_vec[2]*t + line[i][2]
        contact_append = np.array([[contact_x,contact_y,contact_z]])
        
        contact = np.append(contact,contact_append)

    # contact는 두개의 투영된 점이 만드는 선 [[x1,y1,z2],[x2,y2,z2]] 직선의 교점을 구해야한다 find_contact이용
    
    contact_point = np.array([[]])

    cp = find_contact(v1, contact)
    if(cp is not None):
        contact_point = np.append(contact_point,cp)
    cp = find_contact(v2, contact)
    if(cp is not None):
        contact_point = np.append(contact_point,cp)
    cp = find_contact(v3, contact)
    if(cp is not None):
        contact_point = np.append(contact_point,cp)
    cp = find_contact(v4, contact)
    if(cp is not None):
        contact_point = np.append(contact_point,cp)

    contact_point = contact_point.reshape(-1,3)

    for i in range(contact_point.shape[0]):
        pil(v1,contact_point[i])



def find_contact(l1, l2):
    # 직선 두개가 접하는지 검사하는 함수 return 값은 접점 np.array[x,y,z] 만나지 않으면 None을 리턴
    # input  l1 np.array([x1,y1,z1], [x2,y2,z2])
    #        l2 np.array([x1,y1,z1], [x2,y2,z2])

    v1 = l1[0]-l1[1]    
    #v1 = v1/np.linalg.norm(v1)

    v2 = l2[0]-l2[1]
    #v2 = v2/np.linalg.norm(v2)

    if(v1[0]==0):
        t2 = (l1[0][0]-l2[0][0])/v2[0]
        t1 = (l2[0][1]-l1[0][1]+v2[1]*t2)/v1[1]

    if((v2[1]-v1[1]*v2[0]/v1[0]) == 0 ):
        return None
    
    else:
        t2 = (-v1[1]/v1[0]*l1[0][0]+l1[0][1]-l2[0][1]+v1[1]/v1[0]*l2[0][0])/(v2[1]-v1[1]*v2[0]/v1[0])
        t1 = (l2[0][0]-l1[0][0]+v2[0]*t2)/v1[0]

    z1 = l1[0][2]+v1[2]*t1
    z2 = l2[0][2]+v2[2]*t2

    if(abs(z1-z2)<0.1):

        x = l1[0][0]+v1[0]*t1
        y = l1[0][1]+v1[1]*t1
        return np.array([x,y,z1])

    else:
        return None

def pil(l,p):
    # point in line 해당 포인트가 선분 안에 있는지 검사 안에 있으면 True 아니면 False 리턴
    #input l np.array([x1,y1,z1], [x2,y2,z2])
    #      p np.array([x,y,z])

    if((l[0][0] >= p[0] and l[1][0] <= p[0]) or (l[1][0] >= p[0] and l[0][0] <= p[0])):
        if((l[0][1] >= p[1] and l[1][1] <= p[1]) or (l[1][1] >= p[1] and l[0][1] <= p[1])):
            if((l[0][2] >= p[2] and l[1][2] <= p[2]) or (l[1][2] >= p[2] and l[0][2] <= p[2])):
                return True
    
    else:
        return False

def vertices2surface(vertices):
    # vertices = np.araay (8,3) 을 받으면 (posx2line을 써서 나온 값) 6개의 면을 가진 surface = np.array(6,4,3)을 반환
    surface = np.array([
    [vertices[1], vertices[3], vertices[5], vertices[7]], # Bottom face
    [vertices[0], vertices[2], vertices[4], vertices[6]], # Top face
    [vertices[7], vertices[6], vertices[4], vertices[5]], # Front face
    [vertices[1], vertices[0], vertices[2], vertices[3]], # Back face
    [vertices[1], vertices[0], vertices[6], vertices[7]], # Right face
    [vertices[3], vertices[2], vertices[4], vertices[5]]  # Left face
        ])

    return surface

def surface2vertices(surface):
    # 6개의 면을 가진 surface = np.array(6,4,3)을 받아 vertices = np.araay (8,3) 반환
    
    vertices = np.array([surface[1][0],surface[0][0],surface[1][1],surface[0][1],surface[1][2],surface[0][2],surface[1][3],surface[0][3]])

    return vertices




def surface_points_divide(surface, points):
    #surface가 points를 나누는지(divide) 검사하는 함수
    #나누면 True 안나누면 False return
    #surface는 np array 4개의 점 points는 n개의 점

    #surface에서 임의의 세개의 점들을 뽑아 외적을 통해 normal 벡터를 구한다 크기는 규격화 x   + surface를 이루는 모든 line을 정의한다

    v1 = surface[0]-surface[1]
    v2 = surface[1]-surface[2]
    v3 = surface[2]-surface[3]
    v4 = surface[3]-surface[0]


    normal_vec = np.cross(v1,v2) #[x1,y1,z1] 평면 방정식은 normal_vec[0]x + normal_vec[1]y + normal_vec[2]z + d -= 0;
    d = -np.dot(normal_vec,surface[0])

    # 점 하나하나 검사

    # 양수면 True 음수면 False인 bool_list
    bool_list = []


    for i in range(points.shape[0]):
        # 점을 평면에 투영

        t = -(np.dot(normal_vec,points[i])+d)/np.dot(normal_vec,normal_vec)
        x = normal_vec[0]*t + points[i][0]
        y = normal_vec[1]*t + points[i][1]
        z = normal_vec[2]*t + points[i][2]

        point = np.array([x,y,z]) # point: points가 surface에 투영되는 점

        point_surface_vector= np.array([[]])

        for j in range(surface.shape[0]):
            point_surface_vector = np.append(point_surface_vector, point - surface[j])     # point가 끝점, surface가 시작점인 벡터 4개
        point_surface_vector = point_surface_vector.reshape(4,3)

        space_in = 'out'
        
        radian = 0

        for j in range(point_surface_vector.shape[0]):
            cos_theta = np.dot(point_surface_vector[j-1], point_surface_vector[j]) / \
                        (np.linalg.norm(point_surface_vector[j-1]) * np.linalg.norm(point_surface_vector[j]))
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            radian += np.arccos(cos_theta)
            
        if (radian >=2*3.14):
            space_in = 'in'
        
        if(space_in == 'in'):
            # 평면의 정사영 안에 있는 점들 뽑음. points무리를 surface가 나누는지 검사

            # ax.scatter(points[i][0], points[i][1], points[i][2], color="orange", s=20)

            initial_value = np.dot(normal_vec,points[i]) + d

            if(initial_value >= 0):
                bool_list.append(True)

            else:
                bool_list.append(False)

    # all true 인가?
    if(all(bool_list)):
        return False
    # all False 인가?
    if (all(not value for value in bool_list)):
        return False

    else:
        return True


def target_posx(vertices):
    # target은 8개의 점 np.array(8,3)
    # target을 기준으로 posx 형성
    # posx는 np array (12,6) ,각 면들 * xy yx에 대한 xyzrpy값

    # target을 6개의 면으로 바꾼다
    box = np.array([
    [vertices[0], vertices[1], vertices[2], vertices[3]], # Bottom face
    [vertices[4], vertices[5], vertices[6], vertices[7]], # Top face
    [vertices[0], vertices[1], vertices[5], vertices[4]], # Front face
    [vertices[2], vertices[3], vertices[7], vertices[6]], # Back face
    [vertices[1], vertices[2], vertices[6], vertices[5]], # Right face
    [vertices[4], vertices[7], vertices[3], vertices[0]]  # Left face
    ])

    # target의 중심 center 

    posx = np.array([])
    xyz = np.array([])

    center = np.array([0,0,0])

    for i in range(box.shape[0]):
        for j in range(box.shape[1]):
            center = center + box[i][j]

    center = center/24 # 이 값을 xyz값으로 쓰자! rpy는 각 면들로부터 얻자

    #target[0],[1],[2],[3],[4],[5]
    for i in range(box.shape[0]):
        #target[i] 면의 중심을 찾아 z축 정하고 xy, yx를 정한다
        surface_center = np.array([0,0,0])

        for j in range(box.shape[1]):
            surface_center = surface_center + box[i][j]

        surface_center = surface_center/box.shape[1]
        # surface_center confirm

        zvec = center - surface_center
        z1 = 2 * np.linalg.norm(zvec)

        zunit = zvec/np.linalg.norm(zvec)
        
        xvec = box[i][0]-box[i][1]
        x1 = np.linalg.norm(xvec)
        xunit = xvec/np.linalg.norm(xvec)

        yvec = box[i][1]-box[i][2]
        y1 = np.linalg.norm(yvec)
        yunit = np.cross(zunit, xunit)


        R= np.array([xunit, yunit,zunit]).T
        y,p,r = get_rpy(R)

        posx = np.append(posx, [center[0], center[1], center[2], y,p,r])
        xyz = np.append(xyz, [x1,y1,z1])
  
        R= np.array([yunit, -xunit,zunit]).T
        y,p,r = get_rpy(R)

        posx = np.append(posx, [center[0], center[1], center[2], y,p,r])
        xyz = np.append(xyz, [y1,x1,z1])


    posx = posx.reshape(12,-1)
    xyz = xyz.reshape(12, -1)
    return posx, xyz



def obstacle_volume(obstacle, volume):
    # volume이 obstalce을 등분하는가 등분하면 True 등분 안하면 False 반환
    # 장애물 (여러개의 면) np.array (n,4,3)
    # 볼륨,그리퍼 (4개의 선) np.array (4,2,3), posx2line 순서
    # volume이 obstacle을 검사하고 obstacle도 volume을 검사해야 한다.

    # 볼륨을 면으로 만들어 각각의 면을 검사하자

    terminal_width = shutil.get_terminal_size().columns

    surfaces = vertices2surface(volume.reshape(8,3))



    surface_inspect_obstacle = False
    for i in range(surfaces.shape[0]): # 한개의 서피스에 대해 검사
        for j in range(obstacle.shape[0]): # 한개의 obstacle 면에 대해 검사
            divide = surface_points_divide(surfaces[i], obstacle[j])
            if(divide):
                surface_inspect_obstacle = True
                break
        if(surface_inspect_obstacle):
            break


    obstalce_inspect_surface = False
    for i in range(obstacle.shape[0]): # 한개의 obstacle 면이 volume 검사
        for j in range(surfaces.shape[0]) :
            divide = surface_points_divide(obstacle[i], surfaces[j])
            if(divide):
                obstalce_inspect_surface = True
                break

        if(obstalce_inspect_surface):
            break

    if(surface_inspect_obstacle or obstalce_inspect_surface):
        return True

    else:
        return False


def integrated_pick(pub, obstacle, vertices,names, volumes,posxBox, suc_target):
    # 종합적으로 순서를 지어주는 함수
    # obstacle은 바닥과 상자의 면같은 장애물 (n,4,3)
    # vertices는 8개의 점으로 오는 물체의 갯수 (n,8,3)
    # volumes는 자세에 대한 로봇 체적  기준점의 평행이동 --> xyz,xyz <-- 체적 xyz (그리퍼, 조인트, 체적) (n,6)
    # 리턴값 final_posx는 모든 물체의 posx를 추출한다
    # 터미널 너비 가져오기
    print(f" integrated pick에서 names : {names}")
    terminal_width = shutil.get_terminal_size().columns
    print("--------integrated_pick--------".center(terminal_width, "-"))
    
    msg = pick_plot()
    msg.obstacle = obstacle.reshape(-1).tolist()
    msg.vertices = vertices.reshape(-1).tolist()
    msg.volumes = volumes.reshape(-1).tolist()


    # while 문으로 suc_target의 갯수가 vertices.shape[0]과 다른 동안
    change = True
    suc = False
    while(suc_target.shape[0] != vertices.shape[0]):

        if(not change):
            break
        # 가능한 물체가 없으면 while문 탈출을 위해 change = False라는 플래그 설정
        change = False
        if(suc):
            break

        for target in range(vertices.shape[0]):

            # 만약에 target의 posx를 이미 찾았다면 다음 타겟으로 넘어가자

            if np.isin(target,suc_target):
                continue

            # vertices중 타겟을 먼저 정하자 vertices[target]
            # 타겟 기준으로 posx 12개를 구하자
            posx, target_xyz = target_posx(vertices[target])

            if(suc):
                break

            # 12개의 posx로 가능한 volumes의 자세를 만들자 그전에 각도에 대해 2pi를 50등분하자
            for theta_n in range(50):
                # 만약 자세를 찾으면 다음 타겟으로 이동하기 위한 플래그 suc를 이용하자
                if(suc):
                    break
                # theta 만큼 회전
                theta = theta_n * 2 * np.pi / 50

                

                for posx_n in range(posx.shape[0]):
                    # n번째 각도의 n번째 posx를변환
                    # p1 은 센터 위치 p2는 xunit의 끝 point 는 zunit의 끝

                    # unitVec는 posx에 해당하는 unitVec =[[xunit],[yunit],[zunit]]


                    
                    unitVec = rpy2vector(posx[posx_n][3],posx[posx_n][4],posx[posx_n][5])


                    p1 = np.array([posx[posx_n][0],posx[posx_n][1],posx[posx_n][2]])
                    p2 = p1 + unitVec[0]

                    point = p1 + unitVec[2]
                    rotated_point = rotate_point_around_axis(p1, p2, point, theta)

                    rotated_zunit = rotated_point - p1
                    rotated_yunit = np.cross(rotated_zunit, unitVec[0])

                    R= np.array([unitVec[0], rotated_yunit, rotated_zunit]).T
                    y,p,r = get_rpy(R)

                    increased_posx = np.array([p1[0],p1[1],p1[2],y,p,r])

                    # n번째의 볼륨에 대해 충돌 검사
                    for volume_n in range(volumes.shape[0]):
                        volume = posx2line(increased_posx, volumes[volume_n])

                        obstacle2 = obstacle.copy()
                        
                        for vertices_n in range(vertices.shape[0]):
                            # vertices_n이 target이 아니면 + suc_target안에 없으면 surfaces로 만들어 obstacle2에 추가한다.
                            
                            if(vertices_n != target and not np.isin(vertices_n, suc_target)):
                                surfaces = np.array([
                                [vertices[vertices_n][0], vertices[vertices_n][1], vertices[vertices_n][2], vertices[vertices_n][3]], # Bottom face
                                [vertices[vertices_n][4], vertices[vertices_n][5], vertices[vertices_n][6], vertices[vertices_n][7]], # Top face
                                [vertices[vertices_n][0], vertices[vertices_n][1], vertices[vertices_n][5], vertices[vertices_n][4]], # Front face
                                [vertices[vertices_n][2], vertices[vertices_n][3], vertices[vertices_n][7], vertices[vertices_n][6]], # Back face
                                [vertices[vertices_n][1], vertices[vertices_n][2], vertices[vertices_n][6], vertices[vertices_n][5]], # Right face
                                [vertices[vertices_n][4], vertices[vertices_n][7], vertices[vertices_n][3], vertices[vertices_n][0]]  # Left face
                                ])

                                for surface_n in range(surfaces.shape[0]):
                                    obstacle2 = np.append(obstacle2, surfaces[surface_n])

                        obstacle2 = obstacle2.reshape(-1,4,3)
                        result_bool = obstacle_volume(obstacle2, volume)
                        if(result_bool):
                            break
                        
                        
                    if(not result_bool):
                        isMove1 = movejx(increased_posx, vel= 10, acc = 10, time = 0, object_z = -(target_xyz[posx_n][2]/2 + 30))

                        if(not isMove1):
                            continue
                        # 얕게 들어가서 increased_posx를 1cm 더 깊게 
                        increased_posx = posx_calculate(increased_posx, object_x=0,object_y=0,object_z=10)
                        isMove2 = movejx(increased_posx, vel= 10, acc = 10, time = 0)


                        if(not isMove2):
                            movejx(posxBox, vel =10, acc = 10, time = 0, object_z = -200)
                            continue;    

                        if(isMove1 and isMove2):
                            move_gripper(0, 10)
                            movejx(increased_posx, vel= 10, acc = 10, time = 0, object_z = -(target_xyz[posx_n][2]/2 + 30))
                            final_posx = increased_posx
                            suc_target = np.append(suc_target, target)
                            xyz = target_xyz[posx_n]
                            
                            msg.posx = final_posx.reshape(-1).tolist()
                            pub.publish(msg)

                            change = True
                            suc = True
                            break


    if(suc):
        xyz = xyz.reshape(-1,3)

    # print("*****input*****".center(terminal_width, "*"))


    # print("final_posx = ", final_posx)
    # print("suc_target = ", suc_target)
    # print('xyz = ', xyz)
    # print("---------------".center(terminal_width, "-"))
    print(f" integrated pick 계산 완료 후 변화된 names, 즉 물체를 잡는 순서")
    suc_target = suc_target.astype(np.int8)
    print('name = ', names[suc_target[-1]])
    return final_posx, suc_target, xyz, theta, suc


def point3_plane(p1, p2, p3, point):
    # 3개의 포인트 p1 p2 p3를 받아 평면을 만들고 point를 지나는 posx xyzrpy를 추출한다
    # 그리퍼의 끝단 기준으로 posx를 추출하기 때문에 이 함수를 쓰는 곳에서 캠 기준으로 바꿔야 한다
    # x축은 p1,p2 상에, y축이 +z쪽, -z쪽 두개로 향해있게 둔다 즉 두개의 posx가 추출됨
    # 법선 벡터는 항상 point 방향으로 향해 있어야 한다

    v1 = p1 - p2
    v2 = p2 - p3

    normal_vec = np.cross(v1,v2)

    direction = p1 - point
    if (np.dot(normal_vec,direction) < 0):
        normal_vec = -normal_vec
    
    zunit = normal_vec/np.linalg.norm(normal_vec)

    xunit = v1/np.linalg.norm(v1)

    yunit = np.cross(zunit,xunit)

    R = np.array([xunit,yunit,zunit]).T
    y,p,r = get_rpy(R)

    posx1 = np.array([point[0], point[1], point[2], y, p, r])

    R = np.array([-xunit, -yunit, zunit]).T
    y,p,r = get_rpy(R)

    posx2 = np.array([point[0], point[1], point[2], y, p, r])

    posx = np.array([posx1, posx2])

    return posx
    




def integrated_servoing(pub, obstacle, vertices,names, volumes, cam, posxBox):
    # 환경을 인식받아 servoing posx를 추출해주는 함수
    # obstacle, vertices, volumes는 integrated_pick과 같은 형식으로 받고 cam 은 카메라와 그리퍼 끝단의 평행이동 정도 x,y,z (-로)
    # 최종 servoing_posx 는 물체들의 모든 servoing_pos를 출력한다
    # servoing_pos는 자세가 바뀌는게 아닌 거리를 띄워놓는 형식으로 만든다 이때 servoing_pos는 그리퍼 끝단 기준으로 모든 물체에 대해 출력한다
    terminal_width = shutil.get_terminal_size().columns
    print("--------integrated_servoing--------".center(terminal_width, "-"))
    
    msg = servoing_plot()
    msg.obstacle = obstacle.reshape(-1).tolist()
    msg.vertices = vertices.reshape(-1).tolist()
    msg.volumes = volumes.reshape(-1).tolist()

    servoing_posx = np.array([])
    servoing_point = np.array([])
    servoing_result = np.array([])

    # 8개의 방향에서 48개의 자세로 봐야 하므로
    # 0번 모서리 방향에서 볼때 p1 p2 p3 = 1 3 4, 3 4 1, 4 1 3
    # 1번 모서리 방향에서 볼때 p1 p2 p3 = 0 2 5, ...
    # 2번 모서리 방향에서 볼때 p1 p2 p3 = 1 3 6
    # 3번 모서리 방향에서 볼때 p1 p2 p3 = 0 2 7
    # 4번 모서리 방향에서 볼때 p1 p2 p3 = 5 7 0
    # 5번 모서리 방향에서 볼때 p1 p2 p3 = 4 6 1
    # 6번 모서리 방향에서 볼때 p1 p2 p3 = 5 7 2
    # 7번 모서리 방향에서 볼때 p1 p2 p3 = 4 6 3

    for target in range(vertices.shape[0]):

        posx00 = point3_plane(vertices[target][1], vertices[target][3], vertices[target][4], vertices[target][0])
        posx01 = point3_plane(vertices[target][3], vertices[target][4], vertices[target][1], vertices[target][0])
        posx02 = point3_plane(vertices[target][4], vertices[target][1], vertices[target][3], vertices[target][0])

        posx10 = point3_plane(vertices[target][0], vertices[target][2], vertices[target][5], vertices[target][1])
        posx11 = point3_plane(vertices[target][2], vertices[target][5], vertices[target][0], vertices[target][1])
        posx12 = point3_plane(vertices[target][5], vertices[target][0], vertices[target][2], vertices[target][1])

        posx20 = point3_plane(vertices[target][1], vertices[target][3], vertices[target][6], vertices[target][2])
        posx21 = point3_plane(vertices[target][3], vertices[target][6], vertices[target][1], vertices[target][2])
        posx22 = point3_plane(vertices[target][6], vertices[target][1], vertices[target][3], vertices[target][2])


        posx30 = point3_plane(vertices[target][0], vertices[target][2], vertices[target][7], vertices[target][3])
        posx31 = point3_plane(vertices[target][2], vertices[target][7], vertices[target][0], vertices[target][3])
        posx32 = point3_plane(vertices[target][7], vertices[target][2], vertices[target][0], vertices[target][3])


        posx40 = point3_plane(vertices[target][5], vertices[target][7], vertices[target][0], vertices[target][4])
        posx41 = point3_plane(vertices[target][7], vertices[target][0], vertices[target][5], vertices[target][4])
        posx42 = point3_plane(vertices[target][0], vertices[target][5], vertices[target][7], vertices[target][4])

        posx50 = point3_plane(vertices[target][4], vertices[target][6], vertices[target][1], vertices[target][5])
        posx51 = point3_plane(vertices[target][6], vertices[target][1], vertices[target][4], vertices[target][5])
        posx52 = point3_plane(vertices[target][1], vertices[target][4], vertices[target][6], vertices[target][5])

        posx60 = point3_plane(vertices[target][5], vertices[target][7], vertices[target][2], vertices[target][6])
        posx61 = point3_plane(vertices[target][7], vertices[target][2], vertices[target][5], vertices[target][6])
        posx62 = point3_plane(vertices[target][2], vertices[target][5], vertices[target][7], vertices[target][6])

        posx70 = point3_plane(vertices[target][4], vertices[target][6], vertices[target][3], vertices[target][7])
        posx71 = point3_plane(vertices[target][6], vertices[target][3], vertices[target][4], vertices[target][7])
        posx72 = point3_plane(vertices[target][3], vertices[target][4], vertices[target][6], vertices[target][7])

        posx = np.array([posx00, posx01, posx02, 
                        posx10, posx11, posx12,
                        posx20, posx21, posx22,
                        posx30, posx31, posx32,
                        posx40, posx41, posx42,
                        posx50, posx51, posx52,
                        posx60, posx61, posx62,
                        posx70])

        posx = posx.reshape(-1,6)

        # 120mm 기본으로 10mm씩 retreat해서 자세를 찾자
        # 진짜 끝까지 가도 안되면 어떡하지??

        count = 12  
        findPosx = False
        while(not findPosx):

            for posx_n in range(posx.shape[0]):
                # 먼저 cam 위치 + count 후진으로 posx를 평행이동 시키자
                cam_posx = posx_calculate(posx[posx_n], -cam[0], -cam[1], -cam[2] -100- 10 * count)
                
                #해당 자세의 충돌 검사 먼저 볼륨을 형성하자
                for volume_n in range(volumes.shape[0]):
                    print('volume_n = ', volume_n)
                    volume = posx2line(cam_posx, volumes[volume_n])
                
                    # n번째의 볼륨에 대해 장애물을 만들자 원래 장애물 obstacle 등록후
                    obstacle2 = obstacle.copy()

                    for vertices_n in range(vertices.shape[0]):
                        # vertices_n이 target이 아니면 + suc_target안에 없으면 surfaces로 만들어 obstacle2에 추가한다.
                        
                        surfaces = np.array([
                        [vertices[vertices_n][0], vertices[vertices_n][1], vertices[vertices_n][2], vertices[vertices_n][3]], # Bottom face
                        [vertices[vertices_n][4], vertices[vertices_n][5], vertices[vertices_n][6], vertices[vertices_n][7]], # Top face
                        [vertices[vertices_n][0], vertices[vertices_n][1], vertices[vertices_n][5], vertices[vertices_n][4]], # Front face
                        [vertices[vertices_n][2], vertices[vertices_n][3], vertices[vertices_n][7], vertices[vertices_n][6]], # Back face
                        [vertices[vertices_n][1], vertices[vertices_n][2], vertices[vertices_n][6], vertices[vertices_n][5]], # Right face
                        [vertices[vertices_n][4], vertices[vertices_n][7], vertices[vertices_n][3], vertices[vertices_n][0]]  # Left face
                        ])

                        for surface_n in range(surfaces.shape[0]):
                            obstacle2 = np.append(obstacle2, surfaces[surface_n])

                    obstacle2 = obstacle2.reshape(-1,4,3)
                    result_bool = obstacle_volume(obstacle2, volume)

                    if(result_bool):
                        break



                if(not result_bool):
                    isMove1 = movejx(cam_posx, vel = 15, acc = 10, time = 0, object_z = -50)
                    if(not isMove1):
                        continue
                    
                    if(isMove1):
                        isMove2 = movejx(cam_posx, vel = 15, acc = 10, time = 0)
                        
                        if(not isMove2):
                            movejx(posxBox, vel  = 15, acc = 10, time = 0, object_z = -200)
                            continue
                        
                        if(isMove2):
                            print(names[target])

                            servoing_posx = cam_posx
                            findPosx = True
                            servoing_point = np.array(vertices[target][int(posx_n/6)])
                            msg.posx = servoing_posx.reshape(-1).tolist()
                            msg.servoing_point = servoing_point.reshape(-1).tolist()
                            pub.publish(msg)
                            # !!!!!!!!!!!!!!!! servoing 서비스 !!!!!!!!!!!!
                            print(vertices[target].reshape(-1))
                            correct_vertices = servoing_func(vertices[target].reshape(-1))
                            servoing_result = np.append(servoing_result, correct_vertices)
                            movejx(cam_posx, vel = 15, acc = 10, time = 0, object_z = -50)
                            
                            
                            
                            # print("posx_xyz = ", posx[posx_n][0], posx[posx_n][1], posx[posx_n][2])
                            # ax.scatter(posx[0][0],posx[0][1],posx[0][2], color="red", s=200, label="posx")
                            # ax.scatter(*vertices[target][int(posx_n/2)], color="blue", s = 50)
                            movejx(posxBox, vel  = 15, acc = 10, time = 0, object_z = -200)

                            break
            count += 1
            
    servoing_result = servoing_result.reshape(-1,8,3)


    print("servoing_posx = ", servoing_posx)
    print("---------------".center(terminal_width, "-"))

    return servoing_posx, servoing_result
    







def vision_vertices_client():
    # 비전 정보로 물체 좌표를 받아오는 함수
    print("waiting vision_vertices...")
    rospy.wait_for_service('/collision_detect/vision_vertices')

    try:
        vision_vertices_service = rospy.ServiceProxy('/collision_detect/vision_vertices', vision_vertices)
        response = vision_vertices_service()  # 요청 전송, 별도 요청 데이터 없음
        # rospy.loginfo("Received data: %s", response.data)
        
        vertices = np.array(response.vertices)

        vertices = vertices.reshape(-1,8,3)
        names = response.names
        print(names)
        data_dict = {name: vertex for name, vertex in zip(names, vertices)}
        # for name, vertices in zip(names, vertices):
        #     print(f"Name : {name}")
        #     print("vertices : ")
        #     print(vertices)
        terminal_width = shutil.get_terminal_size().columns
        # print("----vision_vertices----".center(terminal_width, "-"))
        # print("vertices  = ", vertices)
        # print("---------------".center(terminal_width, "-"))

        return vertices, names
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)



# 메인 코드

rospy.init_node('collision_detect', anonymous=True)



velB = 20
accB = 20

posxBox = [500,500, 400, 0, 180, 0]

movejx(posxBox, vel =velB, acc = accB, time =0)

[6]
servoing_pub = rospy.Publisher('/collision_detect/servoing_plot', servoing_plot, queue_size=10)
    
pick_pub = rospy.Publisher('/collision_detect/pick_plot', pick_plot, queue_size=10)

pickBox = np.array([   
                    
    # 오른쪽 아래
    [-127.32172, 550.04443, 100],
    [468.93512, 560.19818, 100],
    [464.13327, 854.77448, 100],
    [-130.28603, 849.80872, 100],
    [-127.32172, 550.04443, 0],
    [468.93512, 560.19818, 0],
    [464.13327, 854.77448, 0],
    [-130.28603, 849.80872, 0]
    
    
    ])

pickBox_center = np.sum(pickBox, axis =0)/8

posxPickBox = [pickBox_center[0],pickBox_center[1],pickBox_center[2],0,180,0]

wall1_p1 = pickBox[0]
wall1_p2 = pickBox[1]
wall1_p3 = pickBox[5]
wall1_p4 = pickBox[4]

wall2_p1 = pickBox[1]
wall2_p2 = pickBox[2]
wall2_p3 = pickBox[6]
wall2_p4 = pickBox[5]

wall3_p1 = pickBox[2]
wall3_p2 = pickBox[3]
wall3_p3 = pickBox[7]
wall3_p4 = pickBox[6]

wall4_p1 = pickBox[3]
wall4_p2 = pickBox[0]
wall4_p3 = pickBox[4]
wall4_p4 = pickBox[7]

plane2_p1 = (-500,-500,0)
plane2_p2 = (1500, -500 ,0)
plane2_p3 = (1500,1000, 0)
plane2_p4 = (-500,1000,0)

floor2_p1 = (-500,-500,-100)
floor2_p2 = (1500, -500 ,-100)
floor2_p3 = (1500,1000, -100)
floor2_p4 = (-500,1000,-100)



wall1 = np.array([[wall1_p1,wall1_p2,wall1_p3,wall1_p4]])
wall2 = np.array([wall2_p1, wall2_p2, wall2_p3, wall2_p4])
wall3 = np.array([wall3_p1, wall3_p2, wall3_p3, wall3_p4])
wall4 = np.array([wall4_p1, wall4_p2, wall4_p3, wall4_p4])  
floor = np.array([[plane2_p1, plane2_p2, plane2_p3, plane2_p4]])
floor2 = np.array([floor2_p1,floor2_p2,floor2_p3,floor2_p4])

obstacle = np.array([])
obstacle = np.append(obstacle, wall1)
obstacle = np.append(obstacle, wall2)
obstacle = np.append(obstacle, wall3)
obstacle = np.append(obstacle, wall4)
obstacle = np.append(obstacle, floor)
obstacle = np.append(obstacle, floor2)
obstacle = obstacle.reshape(-1,4,3)

vertices, names = vision_vertices_client()
vertices = np.array(vertices).reshape(-1,8,3)

cam = np.array([0 ,-110, -(210)])

# servoing 전용 volume
# volumes = np.array([[[0,0,0],[45,16,-270]], [cam, [50,15, -30]], [[0,0,-210],[70,70,-110]], [cam, [80, 50, 200]], [[0,0,-320], [60,60,-300]], [[0,0,-250],[250,250,150]]]) # 최적화 전
volumes = np.array([[[0,0,0],[45,20,-270]], [cam, [50,15, -30]], [[0,0,-210],[70,70,-110]], [cam, [80, 50, 215]], [[0,0,-320], [60,60,-300]], [[0,0,-250],[250,250,150]]])


servoing_posx, servoing_result = integrated_servoing(servoing_pub, obstacle, vertices, names, volumes, cam, posxPickBox )

movejx(posxBox, vel =velB, acc = accB, time =0)
#pick 전용 volume과 vertices

volumes = np.array([[[0,0,0],[45,20,-270]], [cam, [50,15, -30]], [[0,0,-210],[70,70,-110]], [[0,0,-320], [60,60,-300]],[[0,0,-250],[250,250,150]]])

vertices = servoing_result

suc_target = np.array([])

suc = True
task_finished = True
while(suc or suc_target.shape[0] != servoing_result.shape[0]):
    for target in range(servoing_result.shape[0]):
        # print(f"target : {target}")
        move_gripper(1, 10)
        pickPosx, suc_target, xyz, theta, suc = integrated_pick(pick_pub, obstacle, vertices, names,volumes, posxPickBox, suc_target)
        suc_target = suc_target.astype(np.int8)
        movejx(posxBox, vel =20, acc = 20, time =0)
        target_packing_xyz, target_packing_center_position = packing(xyz, names[suc_target[-1]])
        print(f"pick {names[suc_target[-1]]}")
        print("target_packing_xyz = ", target_packing_xyz)
        print("target_packing_center_position= ", target_packing_center_position)

        # 다른 그리퍼임 조심!
        place0 = np.array([703.85, 101.30, 30.58])
        placeX = np.array([796.26685, -33.55914, 127.39263])
        placeY = np.array([795.40350, 237.94275, 125.70067])
        placeZ = np.array([567.30054, 99.51237, 162.68108])

      
        xvec = placeY - place0
        yvec = placeX - place0
        zvec = place0 - placeZ

        xvec /= np.linalg.norm(xvec)
        yvec /= np.linalg.norm(yvec)
        zvec /= np.linalg.norm(zvec)

        R1 = np.array([-xvec,-yvec,zvec]).T
        Y,P,R = get_rpy(R1)

        placeYPR = np.array([Y, P, R])

        posx00 = [704.1579424631071, 131.95860375563055, 66.91094241490006, 0.7500418061443787, 134.04891608372557, -45.50533895627467]
        
        posxPlacehome = [734.2806958291524, 98.55521433372843, 284.5731424055672, 0.7500418061443787, 134.04891608372557, -45.50533895627467]
        posxPlace0 = [689.8210245282902, 110.36960629680605, 51.79909365794142, 0.7500418061443787, 134.04891608372557, -45.50533895627467]
        
        
        movejx(posxPlacehome, vel = 10, acc = 10, time =0)
        

        posxPlace = posx_calculate(posxPlace0, -(target_packing_center_position[0]+ 30),
                                                         -(target_packing_center_position[1] + 30),
                                                         -(target_packing_center_position[2]-30))
        movejx(posxPlace, vel =15, acc =15, time =0)
        move_gripper(1, 2)
        
        movejx(posxPlace, vel =15, acc = 15, time =0, object_z = -(200-(target_packing_center_position[2]-30)) )
        
        
        # release gripper
        movejx(posxPlacehome, vel =10, acc =10, time =0)
        
        movejx(posxBox, vel =10, acc = 10, time =0)
        move_gripper(1, 10)
        
        if(servoing_result.shape[0] == suc_target.shape[0]):
            break

        


print("task finished")

rospy.spin()
