# by github.com/tawnkramer
import cv2
import os
import sys
import time

from picamera import PiCamera
from picamera.array import PiRGBArray


class Camera:
    def __init__(self, image_w=160, image_h=128, image_d=3, framerate=20, auto_start=False):

        self.resolution = (image_w, image_h)
        self.framerate = framerate

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True
        self.image_d = image_d
        self.camera = None
        if not auto_start:
            return
        # initialize the camera and stream
        self.camera = PiCamera()  # PiCamera gets resolution (height, width)
        self.camera.resolution = self.resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=self.resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="rgb", use_video_port=True)

    def is_closed(self):
        if self.camera is None:
            return True
        return self.camera.closed

    def start_camera(self):
        if not self.is_closed():
            self.close()
        print('Starting PiCamera')
        self.camera = PiCamera()  # PiCamera gets resolution (height, width)
        self.camera.resolution = self.resolution
        self.camera.framerate = self.framerate
        self.rawCapture = PiRGBArray(self.camera, size=self.resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="rgb", use_video_port=True)

    def set_resolution(self, width, height):
        self.camera.resolution = (width, height)

    def capture(self):
        f = next(self.stream)
        frame = f.array
        self.rawCapture.truncate(0)
        if self.image_d == 1:
            frame = cv2.rgb2gray(frame)
        frame = cv2.flip(frame, -1)
        return frame

    def close(self):
        # indicate that the thread should be stopped
        self.on = False
        print('Closing PiCamera')
        time.sleep(.5)
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()

    def save_photo(self, file_name = str(time.time())):
        image = self.capture()
        image = cv2.flip(image, -1)
        file_name = file_name + ".jpg"
        cv2.imwrite(file_name, image)
        print("Saved photo at: " + os.getcwd() + "/" + file_name)


