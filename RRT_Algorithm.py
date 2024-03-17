import cv2
import numpy as np
import math
import random
import argparse
import os

class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

# Check collision
def collision(x1, y1, x2, y2, img):
    color = []
    x = list(np.arange(x1, x2, (x2 - x1) / 100))
    y = list(((y2 - y1) / (x2 - x1)) * (x - x1) + y1)
    for i in range(len(x)):
        color.append(img[int(y[i]), int(x[i])])
    if 0 in color:
        return True  # Collision
    else:
        return False  # No-collision

# Check the  collision with obstacle and trim
def check_collision(x1, y1, x2, y2, img, stepSize):
    _, theta = dist_and_angle(x2, y2, x1, y1)
    x = x2 + stepSize * np.cos(theta)
    y = y2 + stepSize * np.sin(theta)

    # Trim the branch if it's going out of the image area
    hy, hx = img.shape
    if y < 0 or y > hy or x < 0 or x > hx:
        directCon = False
        nodeCon = False
    else:
        # Check direct connection
        if collision(x, y, end[0], end[1], img):
            directCon = False
        else:
            directCon = True

        # Check connection between two nodes
        if collision(x, y, x2, y2, img):
            nodeCon = False
        else:
            nodeCon = True

    return x, y, directCon, nodeCon

# Return dist and angle between new point and nearest node
def dist_and_angle(x1, y1, x2, y2):
    dist = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
    angle = math.atan2(y2 - y1, x2 - x1)
    return dist, angle

# Return the nearest node index
def nearest_node(x, y, node_list):
    temp_dist = []
    for i in range(len(node_list)):
        dist, _ = dist_and_angle(x, y, node_list[i].x, node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# Generate a random point in the image space
def rnd_point(h, l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return new_x, new_y

def draw_circle(event, x, y, flags, param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2, (x, y), 5, (255, 0, 0), -1)
        coordinates.append(x)
        coordinates.append(y)

def RRT_Planning(img, img2, start, end, stepSize):
    h, l = img.shape
    node_list = [0]
    node_list[0] = Nodes(start[0], start[1])
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])

    # Display start and end
    cv2.circle(img2, start, 5, (0, 0, 255), thickness=3, lineType=8)
    cv2.circle(img2, end, 5, (0, 0, 255), thickness=3, lineType=8)

    i = 1
    pathFound = False
    while not pathFound:
        nx, ny = rnd_point(h, l)

        nearest_ind = nearest_node(nx, ny, node_list)
        nearest_x = node_list[nearest_ind].x
        nearest_y = node_list[nearest_ind].y

        # Check direct connection
        tx, ty, directCon, nodeCon = check_collision(nx, ny, nearest_x, nearest_y, img, stepSize)

        if directCon and nodeCon:
            print("Node can connect directly with end")
            node_list.append(i)
            node_list[i] = Nodes(tx,ty)
            node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            node_list[i].parent_x.append(tx)
            node_list[i].parent_y.append(ty)

            cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (255,0,0), thickness=2, lineType=8)

            print("Path has been found")
            for j in range(len(node_list[i].parent_x)-1):
                cv2.line(img2, (int(node_list[i].parent_x[j]),int(node_list[i].parent_y[j])), (int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])), (255,0,0), thickness=2, lineType=8)
            cv2.imwrite("media/"+str(i)+".jpg",img2)
            cv2.imwrite("out.jpg",img2)
            break

        elif nodeCon:
            node_list.append(i)
            node_list[i] = Nodes(tx, ty)
            node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            node_list[i].parent_x.append(tx)
            node_list[i].parent_y.append(ty)
            i = i + 1
            cv2.circle(img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness=3, lineType=8)
            cv2.line(img2, (int(tx), int(ty)), (int(node_list[nearest_ind].x), int(node_list[nearest_ind].y)),
                     (0, 255, 0), thickness=1, lineType=8)
            cv2.imwrite("media/" + str(i) + ".jpg", img2)
            cv2.imshow("sdc", img2)
            cv2.waitKey(1)

        else:
            continue

    # Draw the last line connecting the last node to the end point
    for j in range(len(node_list[i].parent_x) - 1):
        cv2.line(img2, (int(node_list[i].parent_x[j]), int(node_list[i].parent_y[j])),
                 (int(node_list[i].parent_x[j + 1]), int(node_list[i].parent_y[j + 1])), (255, 0, 0),
                 thickness=2, lineType=8)

    cv2.imwrite("out.jpg", img2)
    cv2.imshow("Final Path", img2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Below are the params:')
    parser.add_argument('-p', type=str, default='world2.png', metavar='ImagePath', action='store', dest='imagePath',
                        help='Path of the image containing mazes')
    parser.add_argument('-s', type=int, default=10, metavar='Stepsize', action='store', dest='stepSize',
                        help='Step-size to be used for RRT branches')
    parser.add_argument('-start', type=int, default=[20, 20], metavar='startCoord', dest='start', nargs='+',
                        help='Starting position in the maze')
    parser.add_argument('-stop', type=int, default=[450, 250], metavar='stopCoord', dest='stop', nargs='+',
                        help='End position in the maze')
    parser.add_argument('-select', help='Select start and end points from figure', action='store_true')

    args = parser.parse_args()

    try:
        os.remove("media")
    except:
        print("Dir already clean")
    os.mkdir("media")

    img = cv2.imread(args.imagePath, 0)
    img2 = cv2.imread(args.imagePath)
    start = tuple(args.start)
    end = tuple(args.stop)
    stepSize = args.stepSize

    coordinates = []
    if args.select:
        print("Select start and end points by double-clicking, press 'escape' to exit")
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', draw_circle)
        while 1:
            cv2.imshow('image', img2)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                break
        start = (coordinates[0], coordinates[1])
        end = (coordinates[2], coordinates[3])

    RRT_Planning(img, img2, start, end, stepSize)
