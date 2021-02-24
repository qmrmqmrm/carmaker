#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def main():
    rospy.init_node("sign_detection", anonymous=True)
    rospy.Subscriber("/vds_node_localhost_2218/image_raw",Image, video, queue_size = 2)
    rospy.spin()


def video(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    frame = cv2.resize(cv_image, (1200, 800), interpolation=cv2.INTER_AREA)
    new_frame = create_mask(frame)
    roi_frame = set_roi(new_frame)

    # cv2.polylines(frame, roi_coners, True, (255, 0, 0))

    # here
    circle_frame = detect_circle(roi_frame)
    # cv2.imshow("frame", frame)
    # cv2.imshow("frame", roi_frame)
    # cv2.imshow("circle_frame", circle_frame)
    # k = cv2.waitKey(1)

    # if k == ord('q'):
    #     break
    # elif k == ord('s'):
    #     cv2.waitKey(0)


def create_mask(frame):
    green_mask = np.zeros_like(frame)
    # reframe = frame[:250, 500:850, :]
    # ref = frame[:np.uint32(frame_size[1]/3.5) - 10, np.uint32(frame_size[0]/2.4)-10:np.uint32(frame_size[0]/1.8)+10, :]
    # print(np.uint32(frame_size[0]/2.4)-10)
    b = frame[:, :, 0]
    g = frame[:, :, 1]
    r = frame[:, :, 2]

    green_frame = r > 180

    blue_frame = b > 100
    red_frame = g > 100

    blue_mask = np.zeros_like(frame)
    red_mask = np.zeros_like(frame)
    blue_mask[blue_frame] = 1
    red_mask[red_frame] = 1

    unnecessary_mask = cv2.bitwise_or(1 - blue_mask, 1 - red_mask)
    green_mask[green_frame] = 1
    new_frame = frame * green_mask * unnecessary_mask
    return new_frame


def set_roi(new_frame):
    frame_size = new_frame.shape[1::-1]
    roi_coners = np.array([[(np.uint32(frame_size[0] / 1.4) + 30, 0), (np.uint32(frame_size[0] / 3.43) - 30, 0),
                            (np.uint32(frame_size[0] / 2.18) - 30, np.uint32(frame_size[1] / 3.86)),
                            (np.uint32(frame_size[0] / 1.84) + 30, np.uint32(frame_size[1] / 3.86))]], dtype=np.int32)
    mask = np.zeros_like(new_frame)
    mask = cv2.fillPoly(mask, roi_coners, (255, 255, 255))
    new_frame = cv2.bitwise_and(new_frame, mask)
    new_frame = new_frame[:220, 300:850]
    new_frame = cv2.resize(new_frame, (1200, 800))
    return new_frame


def detect_circle(roi_frame):
    color_ind = np.argwhere(roi_frame)
    trf_light_pub = rospy.Publisher("/traffic_light",Int32,queue_size=1)
    if len(color_ind) > 5:
        try:
            y1, x1, r = np.min(color_ind, axis=0).astype(np.int32)
            y2, x2, r = np.max(color_ind, axis=0).astype(np.int32)
            x1 -= 3
            y1 -= 3
            x2 += 3
            y2 += 3
            cv2.rectangle(roi_frame, (x1, y1), (x2, y2), 255, 1)

            rect_frame = roi_frame[y1:y2, x1:x2, :]
            if not rect_frame.all():
                rect_frame = cv2.medianBlur(rect_frame, 5)
                rect_frame = cv2.cvtColor(rect_frame, cv2.COLOR_BGR2GRAY)
                circles = cv2.HoughCircles(rect_frame, cv2.HOUGH_GRADIENT, 1, 1,
                                           param1=40, param2=10, minRadius=3, maxRadius=20)
                if circles is not None:

                    circles = np.uint16(np.around(circles))
                    for i in circles[0, :]:
                        cv2.circle(rect_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    print("red or yello!!!\n plz slow & stop")
                    trf_light_pub.publish(1)

                else:
                    print("green!!!\n go")
                    trf_light_pub.publish(0)
                cv2.imshow("t", rect_frame)
        except ValueError:
            pass
    else:
        print("go")
        trf_light_pub.publish(0)
    return roi_frame


if __name__ == "__main__":
    main()
