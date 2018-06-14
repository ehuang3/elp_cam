#!/usr/bin/env python
import cv2 as cv
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def undistort(img, cal):
    h,w = img.shape[:2]
    # map1, map2 = cv.fisheye.initUndistortRectifyMap(cal.K, cal.D, np.eye(3), cal.K, DIM, cv2.CV_16SC2)
    # undistorted_img = cv.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Knew = cal.K.copy()
    # Knew[(0,1), (0,1)] = 0.4 * Knew[(0,1), (0,1)]
    K = np.array(cal.K)
    K = np.reshape(K, (3,3))
    # print K
    D = np.array(cal.D)
    # print D

    Knew = K.copy()
    Knew[(0,1), (0,1)] = 0.7 * Knew[(0,1), (0,1)]
    # print Knew

    img_undistorted = cv.fisheye.undistortImage(img, K, D=D, Knew=Knew)

    return img_undistorted


def main():
    rospy.init_node('elp_cam_node')

    # Open video stream.
    device = rospy.get_param('~device', 0)
    video = cv.VideoCapture(device)
    if not video.isOpened():
        video.open()
    video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','J','P','G'))
    # video.set(cv.cv.CV_CAP_PROP_FOURCC, cv.cv.CV_FOURCC('M','J','P','G'))
    video.set(3, 2592)
    video.set(4, 1944)
    # video.set(cv.cv.CV_CAP_PROP_FOURCC, cv.cv.CV_FOURCC('M','J','P','G'))

    # Get camera calibration
    camera_info_yaml = rospy.get_param('~camera_info_yaml', 'path')
    cal = yaml_to_CameraInfo(camera_info_yaml)

    # Publish.
    image_topic = rospy.get_param('~image_topic', '/image_raw')
    image_pub = rospy.Publisher(image_topic, Image, queue_size=10)
    rate = rospy.Rate(30) # 30hz
    bridge = CvBridge()
    while not rospy.is_shutdown():
        ret, frame = video.read()
        frame = undistort(frame, cal)
        img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(img)
        print 'Capture'
        rate.sleep()
    print 'Closing'
    video.release()

if __name__=='__main__':
    main()
