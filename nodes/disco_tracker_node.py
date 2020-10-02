#!/usr/bin/env python
from __future__ import print_function
import time
import Queue
import rospy
import cv2
import numpy as np
import jpeg4py as jpeg
from cv_bridge import CvBridge, CvBridgeError
from blob_finder import BlobFinder

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage


class DiscoTracker(object):

    def __init__(self):
        self.use_compressed = False 
        rospy.init_node('disco_tracker')
        self.image_queue = Queue.Queue()
        self.bridge = CvBridge()
        self.roi = {
                'x': (135, 485), 
                'y': ( 60, 405),
                }
        self.blob_finder = BlobFinder(threshold=20, min_area=2, max_area=200)

        if self.use_compressed:
            self.image_topic = '/raspicam_node/image/compressed'
            self.image_sub = rospy.Subscriber(self.image_topic, CompressedImage, self.on_image, queue_size=1)
        else:
            self.image_topic = '/raspicam_node/image'
            self.image_sub = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)


    def on_image(self,msg):
        self.image_queue.put(msg)

    def run(self):
        queue_get_count = 0
        frame_proc_count = 0
        t0 = time.time()

        while not rospy.is_shutdown():
            image_msg = None
            while True:
                try:
                    image_msg = self.image_queue.get_nowait()
                    queue_get_count += 1
                except Queue.Empty:
                    break
            if image_msg is None:
                continue
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            #print('ok: {}, {}'.format(dt, 1/dt))

            if self.use_compressed:
                image_cmp = np.fromstring(image_msg.data, np.uint8)
                image_bgr = jpeg.JPEG(image_cmp).decode()
            else:
                image_bgr = self.bridge.imgmsg_to_cv2(image_msg,desired_encoding='bgr8')

            n0, n1 = self.roi['x']
            m0, m1 = self.roi['y']
            image_bgr = image_bgr[m0:m1,n0:n1]
            image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

            if frame_proc_count == 0:
                self.blob_finder.bg_image = image
            blob_list, blob_image = self.blob_finder.find(image)

            try:
                max_blob = max([(blob['area'], blob) for blob in blob_list])[1]
            except ValueError:
                max_blob = None

            if max_blob is not None:
                print('cx: {}, cy: {}'.format(max_blob['cx'], max_blob['cy']))
            else:
                print('None')

            dropped_frames = queue_get_count - frame_proc_count - 1
            dropped_fraction = float(dropped_frames)/float(queue_get_count)
            frame_proc_count += 1
            #print('{}, {}, {:1.3f}'.format(frame_proc_count, queue_get_count, dropped_fraction))

            if frame_proc_count%1==0:
                cv2.imshow('image', blob_image)
                cv2.waitKey(1)


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = DiscoTracker()
    node.run()

