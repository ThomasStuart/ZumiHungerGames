from zumi.zumi import Zumi
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import IPython.display
import PIL.Image
import time
from zumi.util.camera import Camera
from zumi.util.screen import Screen
from zumi.personality import Personality
from zumi.protocol import Note


cam = Camera()
font = cv2.FONT_HERSHEY_PLAIN
cam.start_camera()
zumi.mpu.calibrate_MPU()
zumi = Zumi()
timeLeft = 60

lookup = [False, False, False, False, False, False]


def scanQR():
    global timeLeft, lookup
    frame = cam.capture()  # take a picture
    h, w, channel = frame.shape  # Get the height and width of the image in pixels
    decodedObjects = pyzbar.decode(frame)  # Run a function that looks for codes in that frame

    if len(decodedObjects) > 0:  # If the code finds more than one code...
        for obj in decodedObjects:  # For each code...
            print("Found ", obj.type)  # Print the type of code (barcode or QR code)
            data = obj.data.decode("utf-8")  # Decode the message
            print("Message: ", data)  # Print the message
            qr_id, t = parseQRdata(data)
            lookup[qr_id] = True
            timeLeft = timeLeft + int(t)


def parseQRdata(txt):
    ID = txt[0]
    ID = int(ID)
    ID = ID - 1

    s = " "
    i = 0
    for char in txt:
        # print(char)
        if i != 0:
            if char != ',':
                s = s + char
        i = i + 1

    time = int(s)

    return ID, time

def run_command( direction ):
    global zumi
    if direction == "w":
        zumi.forward()
    if direction == "s":
        zumi.reverse()
    if direction == "a":
        zumi.turn_left()
    if direction == "d":
        zumi.turn_right()
    if direction == "q":
        zumi.stop()


def main():
    global timeLeft, zumi
    print("Let the odds be in your favor ...")

    valid_commands = ['w', 's', 'd', 'a', 'q']

    while (timeLeft > 0):
        start = time.time()

        # PUT CONTROLS HERE
        direction = input("Please enter a command: ")
        while( direction not in valid_commands ):
            print("Wrong... try again: ")
            direction = input("please enter valid command: ")
        run_command(direction)


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