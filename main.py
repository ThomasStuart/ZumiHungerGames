from zumi.zumi import Zumi
import datetime
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import IPython.display
import PIL.Image
import time
from zumi.util.camera import Camera

cam = Camera()
font = cv2.FONT_HERSHEY_PLAIN
cam.start_camera()
zumi = Zumi()
timeLeft = 10


def scanQR():
    global timeLeft
    frame = cam.capture()  # take a picture
    h, w, channel = frame.shape  # Get the height and width of the image in pixels
    decodedObjects = pyzbar.decode(frame)  # Run a function that looks for codes in that frame

    if len(decodedObjects) > 0:  # If the code finds more than one code...
        for obj in decodedObjects:  # For each code...
            print("Found ", obj.type)  # Print the type of code (barcode or QR code)
            data = obj.data.decode("utf-8")  # Decode the message
            print("Message: ", data)  # Print the message
            timeLeft = timeLeft + int(data)


def main():
    global timeLeft, zumi
    print("Let the odds be in your favor ...")

    while (timeLeft > 0):
        start = time.time()

        # PUT CONTROLS HERE


        # Calculate how much time it took for you to choose a drive command
        # AND time taken to drive to point
        # timeElapsed = timeToChooseCommand + timeToExecuteCommand
        end = time.time()
        timeElapsed = end - start
        timeLeft = timeLeft - timeElapsed

        print("lost ", timeLeft, "  seconds... Hurry up: ", timeLeft, " left")

        if timeLeft > 0:
            scanQR()

    # Lost Game
    print("You were not the choosen one!")
    cam.close()


if __name__ == "__main__":

    try:
        main()
    finally:
        cam.close()