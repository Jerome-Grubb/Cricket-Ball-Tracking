import numpy as np

from vpython import *
import cv2
import imutils
import quadFit
import temp1
import linReg

# HSV Colours
lo_red = np.array([0, 25, 0])
hi_red = np.array([5, 255, 255])
lo_red2 = np.array([150, 25, 0])
hi_red2 = np.array([180, 255, 255])

# Replace with your own video
video = "example1.mp4"

coordinates_file = open("coordinatesTest.txt", "w")


vs = cv2.VideoCapture(video)
prevY = -float(inf)
prev_bounce = "down"
coordinates_list = []
fl
while True:
    is_bounce = 0
    frame = vs.read()
    frame = frame[1]
    if frame is None:
        break

    # Blur and convert frame to HSV
    frame = imutils.resize(frame, width=1000)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Create mask
    mask1 = cv2.inRange(hsv, lo_red, hi_red)
    mask2 = cv2.inRange(hsv, lo_red2, hi_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Do morphological operations
    kernal = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernal, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal, iterations=4)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal, iterations=4)

    # Uncomment to see mask
    # cv2.imshow('mask', mask)
    # cv2.waitKey()

    # Finds all contours from mask
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    centroid = sorted(cnts, key=cv2.contourArea)
    the_one = None

    for i in centroid:
        ((x, y), radius) = cv2.minEnclosingCircle(i)
        the_one = i
    centroid = the_one

    if centroid is not None:
        ((x, y), radius) = cv2.minEnclosingCircle(centroid)
        if y < prevY:
            if prev_bounce == "down":
                prev_bounce = "up"
                is_bounce = 1
        else:
            is_bounce = 0
        prevY = y
        M = cv2.moments(centroid)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # Checks minimum radius size and puts circle around largest blob
        if radius > 1:
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

        coordinates_list.append((x+25, y+25, radius, is_bounce))

    # Uncomment to see frame with centroid. Press Q to cycle through frames
    # cv2.imshow("Frame", frame)
    # key = cv2.waitKey(20) & 0xFF

vs.release()
cv2.destroyAllWindows()

# Write information to text file. Probably don't need to write it to a file, but whatever
linearReg = linReg.linearRegression(coordinates_list)
quadRegressions = quadFit.quadraticRegression(coordinates_list)
for i in range(0, len(coordinates_list)):
    coordinates_file.write("{:.3f} {:.3f} {:.3f} {} {} {}\n".format(coordinates_list[i][0], coordinates_list[i][1], coordinates_list[i][2], coordinates_list[i][3], linearReg[i], quadRegressions[i]))
coordinates_file.close()


# This is the 'Drawing' section, and the part where the coordinates that were extracted are manipulated

AFTER_BOUNCE_DEFAULT = False
START_RADIUS = 25
END_RADIUS = 12
PITCH_WIDTH = 305.0
PITCH_LENGTH = 2012.0
PITCH_THICKNESS = 10
CREASE_LENGTH = 122
WASTE = 405.4
WICKET_HEIGHT = 71.1
WICKET_WIDTH = 22.86
STUMP_WIDTH = 4.5
WIDE_WIDTH = 264
LINE_WIDTH = 5
BALL_RADIUS = 3.6

# This stuff is just drawing the pitch and stuff in Virtual Python
scene1 = canvas(title="Cricket is cool", width=1280, height=720, range=800, background=vector(0.2, 0.2, 0.2), center=vector(0, 30, 30))
scene1.forward = vector(-1, -0.05, 0.02)
# This draws the pitch
floor = box(pos=vector(0, 0, 0), size=vector(PITCH_LENGTH * 1.2, PITCH_THICKNESS * 1.2, PITCH_WIDTH), color=vector(0.97, 0.94, 0.6))
floor_outer = box(pos=vector(0, 0, 0), size=vector(PITCH_LENGTH * 1.25, PITCH_THICKNESS, PITCH_WIDTH * 2), color=vector(0.2, 0.7, 0.27))
floor_impact = box(pos=vector(0, 0, 0), size=vector(PITCH_LENGTH, PITCH_THICKNESS * 1.3, WICKET_WIDTH), color=vector(0.63, 0.57, 0.93), opacity=0.8)

# This does the crease lines and stumps
batting_wicket1 = box(pos=vector(PITCH_LENGTH / 2, WICKET_HEIGHT / 2, -(WICKET_WIDTH / 2 - STUMP_WIDTH / 2)), size=vector(5, WICKET_HEIGHT, STUMP_WIDTH), color=color.white)
batting_wicket2 = box(pos=vector(PITCH_LENGTH / 2, WICKET_HEIGHT / 2, 0), size=vector(5, WICKET_HEIGHT, STUMP_WIDTH), color=color.white)
batting_wicket3 = box(pos=vector(PITCH_LENGTH / 2, WICKET_HEIGHT / 2, (WICKET_WIDTH / 2 - STUMP_WIDTH / 2)), size=vector(5, WICKET_HEIGHT, STUMP_WIDTH), color=color.white)
line1 = box(pos=vector(PITCH_LENGTH / 2, PITCH_THICKNESS / 2, 0), size=vector(LINE_WIDTH, 5, WIDE_WIDTH), color=color.white)
line2 = box(pos=vector(PITCH_LENGTH / 2, PITCH_THICKNESS / 2, 132), size=vector(244, 5, LINE_WIDTH), color=color.white)
line3 = box(pos=vector(PITCH_LENGTH / 2, PITCH_THICKNESS / 2, -132), size=vector(244, 5, LINE_WIDTH), color=color.white)
line4 = box(pos=vector(PITCH_LENGTH / 2 - 122, PITCH_THICKNESS / 2, 0), size=vector(LINE_WIDTH, 5, 366), color=color.white)

# Draw the other wickets
bowling_wicket1 = box(pos=vector(-PITCH_LENGTH / 2, WICKET_HEIGHT / 2, -(WICKET_WIDTH / 2 - STUMP_WIDTH / 2)), size=vector(5, WICKET_HEIGHT, STUMP_WIDTH), color=color.white)
bowling_wicket2 = box(pos=vector(-PITCH_LENGTH / 2, WICKET_HEIGHT / 2, 0), size=vector(5, WICKET_HEIGHT, STUMP_WIDTH), color=color.white)
bowling_wicket3 = box(pos=vector(-PITCH_LENGTH / 2, WICKET_HEIGHT / 2, (WICKET_WIDTH / 2 - STUMP_WIDTH / 2)), size=vector(5, WICKET_HEIGHT, STUMP_WIDTH), color=color.white)
line1 = box(pos=vector(-PITCH_LENGTH / 2, PITCH_THICKNESS / 2, 0), size=vector(LINE_WIDTH, 5, WIDE_WIDTH),color=color.white)
line2 = box(pos=vector(-PITCH_LENGTH / 2, PITCH_THICKNESS / 2, 132), size=vector(244, 5, LINE_WIDTH), color=color.white)
line3 = box(pos=vector(-PITCH_LENGTH / 2, PITCH_THICKNESS / 2, -132), size=vector(244, 5, LINE_WIDTH),color=color.white)
line4 = box(pos=vector(-PITCH_LENGTH / 2 + 122, PITCH_THICKNESS / 2, 0), size=vector(LINE_WIDTH, 5, 366),color=color.white)

# Define a bunch of lists and variables that we're gonna use later, sorry its a bit messy
SCALE_FACTOR = [1, 0.5, PITCH_LENGTH - (2 * CREASE_LENGTH)]
bouncing_pt = []
xlist = []
ylist = []
rlist = []
balls = []
FY = 1520.0
FX = 350
zlist = []
newx_list = []

# Stores the coordinates for the tracked ball, so pretty important
coords_3d = []

# Open text file and do things with information
with open('testingWizbiz.txt') as coord_file:
    for i, row in enumerate(coord_file):
        x, y, _, is_bouncing_pt, r_new, y_new = row.split()
        newx_list.append((x, y, 0, 0, is_bouncing_pt))
        x = ((float(x) - WASTE) * 305 / 500) - 152
        y = (720.0 - float(y_new)) * SCALE_FACTOR[1]
        bouncing_pt.append(int(is_bouncing_pt))
        if int(is_bouncing_pt) == 1:
            bouncing_pt_idx = i
        xlist.append(x)
        ylist.append(y)
        if i == 0:
            START_RADIUS = float(r_new)
        END_RADIUS = float(r_new)
        rlist.append(float(r_new))

# Creates the z axis based on the radius, truly wild stuff
for i, radius in enumerate(rlist):
    z = abs((START_RADIUS - radius) / (START_RADIUS - END_RADIUS))
    zlist.append(z * SCALE_FACTOR[2])

# Perspective transform calculations, just look it up on Wikipedia
yp = ylist[0] * FY / (FY + zlist[0])
zp = xlist[0] * zlist[0] / (FX + zlist[0])

# Populates the list with the measured coordinates
num_detected_points = len(xlist)
for i in range(len(xlist)):
    yp = ylist[i] * FY / (FY + zlist[i])
    zp = xlist[i] * zlist[i] / (FX + zlist[i])
    coords_3d.append((zlist[i] - ((PITCH_LENGTH - 2 * CREASE_LENGTH) / 2), yp, zp, 1, bouncing_pt[i]))
quadraticReg = quadFit.quadraticRegression(coords_3d, after_bounce_linear=AFTER_BOUNCE_DEFAULT)
linearReg = temp1.quadraticRegression(coords_3d)

# Append the predicted points to the list, but they're just 0 at the moment, as you can seeeeeeeeeeee...
for idx in range(1, 300):
    coords_3d.append(((coords_3d[num_detected_points - 1][0] + idx), 0, 0, 0, 0))

# This is exciting now, this is where the tracked points are plotted into our 3D environment
final_coords_3d = []
for idx in range(num_detected_points):
    final_coords_3d.append((coords_3d[idx][0], quadraticReg[idx], linearReg[idx]))
    if coords_3d[idx][0] > -400:
        # This draws the ball
        balls.append(sphere(pos=vector(coords_3d[idx][0], quadraticReg[idx], linearReg[idx]), radius=BALL_RADIUS,color=vector(0.52, 0.15, 0.19)))
        if idx > 0:
            # This makes the cyclinder thingy
            displacement = vector(final_coords_3d[idx][0] - final_coords_3d[idx - 1][0], final_coords_3d[idx][1] - final_coords_3d[idx - 1][1], final_coords_3d[idx][2] - final_coords_3d[idx - 1][2])
            cylinder(pos=vector(final_coords_3d[idx - 1][0], final_coords_3d[idx - 1][1], final_coords_3d[idx - 1][2]), axis=displacement, radius=BALL_RADIUS, color=vector(0.52, 0.15, 0.19), opacity=0.3)

#  Puts predicted points into a new list yay
for idx in range(0, len(coords_3d)):
    final_coords_3d.append((coords_3d[idx][0], quadraticReg[idx], linearReg[idx]))

# Draws the predicted balls in the 3D environment
STEP_SIZE = 20
for idx in range(num_detected_points, len(coords_3d), 20):
    if coords_3d[idx][0] <= PITCH_LENGTH * 1.2 / 2: # Useless maybe delete idk see how you feel champ
        balls.append(sphere(pos=vector(coords_3d[idx][0], quadraticReg[idx], linearReg[idx]), radius=BALL_RADIUS, color=color.blue, opacity=0.8))
        # Draw the cylinder tube thing
        if idx - STEP_SIZE > 0 and idx > num_detected_points:
            displacement = vector(final_coords_3d[idx][0] - final_coords_3d[idx - STEP_SIZE][0], final_coords_3d[idx][1] - final_coords_3d[idx - STEP_SIZE][1], final_coords_3d[idx][2] - final_coords_3d[idx - STEP_SIZE][2])
            cylinder(pos=vector(final_coords_3d[idx - STEP_SIZE][0], final_coords_3d[idx - STEP_SIZE][1], final_coords_3d[idx - STEP_SIZE][2]), axis=displacement, radius=BALL_RADIUS, color=color.blue, opacity=0.3)

# Yay you made it to the end. If your reading this then make sure you do the report early, do not make the same mistake I did
