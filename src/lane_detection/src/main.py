#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
import glob
import os
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from lane_detection.msg import *
from sensor_msgs.msg import Image


class LaneDetect:
    def __init__(self):
        self.src = None
        self.dst = None
        self.img_size = None
        self.minpixel = 1
        self.leftx_temp = []
        self.lefty_temp = []
        self.rightx_temp = []
        self.righty_temp = []


    def read_video(self, original_frame):
        # cap = cv2.VideoCapture(config.FILE)
        # if not cap.isOpened():
        #     print("file error")

        # while cap.isOpened():
        # _, original_frame = cap.read()

        pub = rospy.Publisher('lane_center_information', lane_center, queue_size = 5)
        # print(original_frame.shape)
        original_frame = original_frame[:-200, :, :]
        original_frame = cv2.resize(original_frame, (1200, 800))
        self.img_size = original_frame.shape[1::-1]
        hls_frame = self.hls_channel(original_frame)
        combined_binary = self.gradient_threshold(hls_frame)

        self.src = np.float32(
            [[(self.img_size[0] / 2) - 80, self.img_size[1] / 2 + 30],  # 좌측 상단
             [(self.img_size[0] * 1/ 6) + 50 , self.img_size[1] - 100],  # 좌측 하단
             [(self.img_size[0] * 5 / 6) - 50 , self.img_size[1] - 100],  # 우측 하단
             [(self.img_size[0] / 2) + 80, self.img_size[1] / 2 + 30]])  # 우측 상단
        # 140 30
        self.dst = np.float32(
            [[(10), 10],
             [(self.img_size[0] *1/ 6) + 50 , self.img_size[1] - 100],
             [(self.img_size[0] * 5 / 6) - 50 , self.img_size[1] - 100],
             [(self.img_size[0] - 10), 10]])

        for x, y in self.src:
            x, y = x.astype(int), y.astype(int)
            cv2.circle(original_frame, (x, y), 1, (255, 0, 0), 3)
        for x, y in self.dst:
            x, y = x.astype(int), y.astype(int)
            cv2.circle(original_frame, (x, y), 1, (0, 0, 255), 3)

        bird_view = self.perspective(combined_binary, dst_size=self.img_size, src=self.src, dst=self.dst)
        left = bird_view[:, :np.int32(bird_view.shape[1] * 1 / 3) + 100]
        right = bird_view[:, np.int32(bird_view.shape[1] * 2 / 3) - 100:-200]
        middle_mask = np.zeros_like(bird_view)
        middle = middle_mask[:, np.int32(middle_mask.shape[1] * 1 / 3) + 100:np.int32(middle_mask.shape[1] * 2 / 3) - 80]
        # lane_frame = np.hstack((middle_mask[:, :200], left))
        lane_frame = np.hstack((left, middle))
        lane_frame = np.hstack((lane_frame, right))
        lane_frame = np.hstack((lane_frame, middle_mask[:, -200:]))

        bird_view_lane, left_fitx, right_fitx = self.fit_polynomial(lane_frame)

        curverad = self.get_curve(original_frame, left_fitx, right_fitx)
        # print(curverad)
        le_shape = left_fitx.shape
        a = np.reshape(left_fitx,(le_shape[0],1))
        b = np.reshape(right_fitx, (le_shape[0], 1))
        c = np.concatenate((a, b), axis=1)
        # zero_one = np.array([[0., 0., 0., 1.]])
        # print(c)
        center_pix = np.mean(c,axis=1)
        # print(center_pix[0],center_pix[-1],center_pix[le_shape[0]/2])
        list_center_pix = [center_pix[0],center_pix[-1],center_pix[le_shape[0]/2]]
        # print(len(list_center_pix),list_center_pix[0])

        # lane_curve = np.mean([curverad[0], curverad[1]])
        color_img = self.draw_lanes(original_frame, left_fitx, right_fitx)
        # cv2.putText(color_img, f"curve:{lane_curve:.2f}", (550, 620), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
        # cv2.putText(color_img, f"offset:{curverad[2]:.2f}", (550, 650), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
        for n, pix in enumerate(center_pix):
            bird_view_lane= cv2.line(bird_view_lane,(int(pix),n),(int(pix),n),(255,255,255),5)
        # cv2.imshow("ss",cen_pixs)
        # cv2.imshow("original_frame", color_img)
        # cv2.imshow("test", bird_view_lane)
        # cv2.waitKey(33)

        # k = cv2.waitKey(33)
        # if k == ord('q'):
        #     cap.release()
        #     break
        # elif k == ord('s'):
        #     cv2.waitKey(0)
        # cv2.destroyAllWindows()

        lane = lane_center()
        # print(lane_curve)
        # print("offset :", curverad[2])
        # print("offset :", (curverad[2]/np.pi)*180)
        # print("lane_curve :", lane_curve)
        # print("left_fitx : ", left_fitx.shape)
        # print("right_fitx : ", right_fitx.shape)
        lane.offset_ = curverad[2]
        lane.center_pix = list_center_pix
        pub.publish(lane)


    def hls_channel(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        return hls

    def gradient_threshold(self, img, sobel_kernel=3, gradient_thresh=(15, 255)):
        h_channel = img[:, :, 0]
        l_channel = img[:, :, 1]
        s_channel = img[:, :, 2]
        sobelxy = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1, ksize=sobel_kernel)
        abs_sobel = np.absolute(sobelxy)
        scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
        gradient_binary = np.zeros_like(scaled_sobel)
        gradient_binary[(scaled_sobel >= gradient_thresh[0]) & (scaled_sobel <= gradient_thresh[1])] = 255
        color_binary = self.color_threshold(s_channel)

        combined_binary = np.zeros_like(gradient_binary)
        combined_binary[(color_binary == 255) | (gradient_binary == 255)] = 255
        return combined_binary

    def color_threshold(self, s_channel, color_thresh=(150, 255)):
        color_binary = np.zeros_like(s_channel)
        color_binary[(s_channel >= color_thresh[0]) & (s_channel <= color_thresh[1])] = 255
        return color_binary

    def perspective(self, img, dst_size=(1280, 720), src=None, dst=None):
        if src is not None:
            M = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(img, M, dst_size, flags=cv2.WARP_FILL_OUTLIERS+cv2.INTER_CUBIC)
            return warped
        else:
            print("check src & dst value")

    def fit_polynomial(self, img):
        leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, out_img = self.find_line(img, draw_window=True)

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])

        try:
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            print("failed to fit a line")
            left_fitx = 1 * ploty ** 2 + 1 * ploty
            right_fitx = 1 * ploty ** 2 + 1 * ploty

        out_img[lefty, leftx] = [255, 0, 100]
        out_img[righty, rightx] = [0, 100, 255]
        return out_img, left_fitx, right_fitx

    def find_line(self, img, nwindow=9, margin=100, draw_window=False):
        out_img = np.dstack((img, img, img))

        histogram = self.get_histogram(img)
        midpoint = int(histogram.shape[0]/2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        # window 높이 설정
        window_height = np.int(img.shape[0]/nwindow)

        nonzero = img.nonzero()
        nonzerox = np.array(nonzero[1])
        nonzeroy = np.array(nonzero[0])

        left_current = left_base
        right_current = right_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindow):
            window_y_low = img.shape[0] - (window+1)*window_height
            window_y_high = img.shape[0] - window*window_height
            window_left_low = left_current - margin
            window_left_high = left_current + margin
            window_right_low = right_current - margin
            window_right_high = right_current + margin

            if draw_window == True:
                cv2.rectangle(out_img, (window_left_low, window_y_low), (window_left_high, window_y_high), (100, 255,
                                                                                                            255), 3)
                cv2.rectangle(out_img, (window_right_low, window_y_low), (window_right_high, window_y_high), (100, 255,
                                                                                                              255), 3)
            good_left_inds = ((nonzeroy >= window_y_low) & (nonzeroy < window_y_high) &
                              (nonzerox >= window_left_low) & (nonzerox < window_left_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= window_y_low) & (nonzeroy < window_y_high) &
                               (nonzerox >= window_right_low) & (nonzerox < window_right_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.minpixel:
                left_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpixel:
                right_current = np.int(np.mean(nonzerox[good_right_inds]))

        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            pass

        # 왼쪽 오른쪽 선 픽셀위치 추출
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        if (len(leftx) > 7000) | (len(lefty) > 7000):
            self.leftx_temp = leftx
            self.lefty_temp = lefty
        else:
            leftx = self.leftx_temp
            lefty = self.lefty_temp
        if (len(rightx) > 3000) | (len(righty) > 3000):
            self.rightx_temp = rightx
            self.righty_temp = righty
        else:
            rightx = self.rightx_temp
            righty = self.righty_temp

        return leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, out_img

    def get_histogram(self, img):
        hist = np.sum(img[img.shape[0] // 2:, :], axis=0)
        return hist

    def draw_lanes(self, img, left_fitx, right_fitx):
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        color_img = np.zeros_like(img)

        left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        points = np.hstack((left, right))

        cv2.fillPoly(color_img, np.int_(points), (0, 200, 255))
        inv_perspective_img = self.inv_perspective(color_img, dst_size=self.img_size, src=self.dst, dst=self.src)
        inv_perspective_img = cv2.addWeighted(img, 1, inv_perspective_img, 0.7, 0)
        return inv_perspective_img

    def inv_perspective(self, img, dst_size=(1280, 720), src=None, dst=None):
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, dst_size)
        return warped

    def get_curve(self, img, leftx, rightx):
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        y_eval = np.max(ploty)
        ym_per_pix = 30.5 / 720  # meters per pixel in y dimension
        xm_per_pix = 3.7 / 720  # meters per pixel in x dimension

        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 2)
        # Calculate the new radii of curvature
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_curverad = ((1 + (
                    2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        car_pos = img.shape[1] / 2
        l_fit_x_int = left_fit_cr[0] * img.shape[0] ** 2 + left_fit_cr[1] * img.shape[0] + left_fit_cr[2]
        r_fit_x_int = right_fit_cr[0] * img.shape[0] ** 2 + right_fit_cr[1] * img.shape[0] + right_fit_cr[2]
        lane_center_position = (r_fit_x_int + l_fit_x_int) / 2
        center = (car_pos - lane_center_position) * xm_per_pix / 10
        # Now our radius of curvature is in meters
        return (left_curverad, right_curverad, center)


    def main(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.read_video(cv_image)


if __name__ == "__main__":
    rospy.init_node("lane_detection", anonymous=True)
    landetect = LaneDetect()


    rospy.Subscriber("/vds_node_localhost_2218/image_raw",Image, landetect.main, queue_size = 2 )
    rospy.spin()

