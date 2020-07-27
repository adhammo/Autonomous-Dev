import cv2
import numpy as np
import math

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def line_pt(r, f, line):
    return (r[0] + f * (line[0][0] - line[1][0]), r[1] + f * (line[0][1] - line[1][1]))


def distance(a, b):
    return math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))


# Read images
img_org = cv2.imread('road_final.jpg')
img_edt = cv2.imread('road_final.jpg')

# Define points
road_pts = [[0, 180], [0, 18], [130, 360], [640, 205]]
rect_pts = [[280, 215], [346, 186], [370, 265], [432, 222]]

#Rectangle escape point
rect_escape_pt = line_intersection((rect_pts[0], rect_pts[1]), (rect_pts[2], rect_pts[3]))
cv2.circle(img_edt, (int(rect_escape_pt[0]), int(rect_escape_pt[1])), 5, (0, 255, 0), cv2.FILLED)
cv2.line(img_edt, (rect_pts[0][0], rect_pts[0][1]), (int(rect_escape_pt[0]), int(rect_escape_pt[1])), (255, 0, 0), 2, cv2.FILLED)
cv2.line(img_edt, (rect_pts[2][0], rect_pts[2][1]), (int(rect_escape_pt[0]), int(rect_escape_pt[1])), (255, 0, 0), 2, cv2.FILLED)

# Rectangle lines
extra_dist = 2.5
extra_pt1 = line_pt(rect_pts[0], extra_dist, (rect_pts[0], rect_pts[2]))
extra_pt2 = line_pt(rect_pts[0], -extra_dist, (rect_pts[0], rect_pts[2]))
extra_pt3 = line_intersection((rect_pts[1], rect_pts[3]), (extra_pt1, rect_escape_pt))
extra_pt4 = line_intersection((rect_pts[1], rect_pts[3]), (extra_pt2, rect_escape_pt))
cv2.line(img_edt, (int(extra_pt1[0]), int(extra_pt1[1])), (int(extra_pt2[0]), int(extra_pt2[1])), (0, 255, 0), 2, cv2.FILLED)
cv2.line(img_edt, (int(extra_pt3[0]), int(extra_pt3[1])), (int(extra_pt4[0]), int(extra_pt4[1])), (0, 255, 0), 2, cv2.FILLED)
cv2.line(img_edt, (int(rect_escape_pt[0]), int(rect_escape_pt[1])), (int(extra_pt1[0]), int(extra_pt1[1])), (0, 255, 0), 2, cv2.FILLED)
cv2.line(img_edt, (int(rect_escape_pt[0]), int(rect_escape_pt[1])), (int(extra_pt2[0]), int(extra_pt2[1])), (0, 255, 0), 2, cv2.FILLED)
cv2.circle(img_edt, (int(extra_pt1[0]), int(extra_pt1[1])), 5, (0, 255, 0), cv2.FILLED)
cv2.circle(img_edt, (int(extra_pt2[0]), int(extra_pt2[1])), 5, (0, 255, 0), cv2.FILLED)
cv2.circle(img_edt, (int(extra_pt3[0]), int(extra_pt3[1])), 5, (0, 255, 0), cv2.FILLED)
cv2.circle(img_edt, (int(extra_pt4[0]), int(extra_pt4[1])), 5, (0, 255, 0), cv2.FILLED)

# Road
cv2.line(img_edt, (road_pts[0][0], road_pts[0][1]), (road_pts[2][0], road_pts[2][1]), (0, 0, 255), 2, cv2.FILLED)
cv2.line(img_edt, (road_pts[1][0], road_pts[1][1]), (road_pts[3][0], road_pts[3][1]), (0, 0, 255), 2, cv2.FILLED)

# Rectangle-Road
rect_road_pts = [line_intersection((road_pts[0], road_pts[2]), (rect_pts[0], rect_pts[1])),
                 line_intersection((road_pts[1], road_pts[3]), (rect_pts[0], rect_pts[1])),
                 line_intersection((road_pts[0], road_pts[2]), (rect_pts[2], rect_pts[3])),
                 line_intersection((road_pts[1], road_pts[3]), (rect_pts[2], rect_pts[3]))]
cv2.line(img_edt, (int(rect_road_pts[0][0]), int(rect_road_pts[0][1])), (int(rect_road_pts[1][0]), int(rect_road_pts[1][1])), (255, 0, 0), 2, cv2.FILLED)
cv2.line(img_edt, (int(rect_road_pts[2][0]), int(rect_road_pts[2][1])), (int(rect_road_pts[3][0]), int(rect_road_pts[3][1])), (255, 0, 0), 2, cv2.FILLED)
for i in range(4):
    cv2.circle(img_edt, (int(rect_road_pts[i][0]), int(rect_road_pts[i][1])), 5, (255, 0, 0), cv2.FILLED)

# Rectangle-Road-Escape
rect_road_escape_pts = [line_intersection((road_pts[0], road_pts[2]), (rect_escape_pt, extra_pt1)),
                        line_intersection((road_pts[1], road_pts[3]), (rect_escape_pt, extra_pt1)),
                        line_intersection((road_pts[0], road_pts[2]), (rect_escape_pt, extra_pt2)),
                        line_intersection((road_pts[1], road_pts[3]), (rect_escape_pt, extra_pt2))]
cv2.line(img_edt, (int(rect_road_escape_pts[0][0]), int(rect_road_escape_pts[0][1])), (int(rect_road_escape_pts[1][0]), int(rect_road_escape_pts[1][1])), (0, 255, 0), 2, cv2.FILLED)
cv2.line(img_edt, (int(rect_road_escape_pts[2][0]), int(rect_road_escape_pts[2][1])), (int(rect_road_escape_pts[3][0]), int(rect_road_escape_pts[3][1])), (0, 255, 0), 2, cv2.FILLED)
for i in range(4):
    cv2.circle(img_edt, (int(rect_road_escape_pts[i][0]), int(rect_road_escape_pts[i][1])), 5, (0, 255, 0), cv2.FILLED)

# Rectangle points
for i in range(4):
    cv2.circle(img_edt, (rect_pts[i][0], rect_pts[i][1]), 5, (0, 0, 255), cv2.FILLED)

# Calculate road width
road_width = 75 * (distance(rect_road_escape_pts[0], rect_road_escape_pts[2]) / distance(rect_pts[0], rect_pts[2]))

# Calculate output size
width, height = int(75 * (distance(rect_road_escape_pts[0], rect_road_escape_pts[1]) / distance(rect_pts[0], rect_pts[1]))), int(road_width)
screen_pts = [[0, 0], [width, 0], [0, height], [width, height]]

# Wrap perspective and output image
matrix = cv2.getPerspectiveTransform(np.float32(rect_road_escape_pts), np.float32(screen_pts))
imgOutput = cv2.warpPerspective(img_org, matrix, (width, height))
cv2.putText(imgOutput, 'Road width: %.2fcm' % road_width, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255))

# Render images
cv2.imshow("Perspective view", img_org)
cv2.imshow("Perspective view lines", img_edt)
cv2.imshow("Bird eye view", imgOutput)

# exit
cv2.waitKey(0)
cv2.destroyAllWindows()
