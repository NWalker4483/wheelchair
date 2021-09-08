import numpy as np
import cv2
# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def find_line_intersection(m1, b1, m2, b2):
    if m1 == m2:
        print ("These lines are parallel!!!")
        return None
    # y = mx + b
    # Set both lines equal to find the intersection point in the x direction
    # m1 * x + b1 = m2 * x + b2
    # m1 * x - m2 * x = b2 - b1
    # x * (m1 - m2) = b2 - b1
    # x = (b2 - b1) / (m1 - m2)

    x = (b2 - b1) / (m1 - m2)
    # Now solve for y -- use either line, because they are equal here
    # y = mx + b
    y = m1 * x + b1
    return x, y
    
def points_to_line(p1, p2):
  # when given two points, return a slope and bias value which will intersect with those given points
  x1, y1 = p1
  x2, y2 = p2
  m = (y2 - y1) / (x2 - x1)
  b = y1 - (m * x1)
  return m, b

def line_to_points(m, b):
  # Return origin and unit vector
  return (0, 0), (1, m + b)

def rotate_about(point, origin, radians):
  x, y = point
  offset_x, offset_y = origin
  adjusted_x = (x - offset_x)
  adjusted_y = (y - offset_y)
  cos_rad = np.cos(radians)
  sin_rad = np.sin(radians)
  qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
  qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
  return qx, qy
    
def get_boxpoints(box_hw):
  return [(0, box_hw[0]), box_hw[::-1], (box_hw[1], 0), (0,0)]

def project_viewbox(dx = 10, dy = 10, dr = .25, box_points = [(0, 1), (1,1), (1,0), (0,0)]):
  box_points = list(map(lambda pt: (pt[0] + dx, pt[1] + dy), box_points))

  box_center = ((box_points[0][0] + box_points[3][0])/2, (box_points[0][1] + box_points[3][1])/2)
  
  box_points = list(map(lambda pt: rotate_about(pt, box_center, dr), box_points))
  
  return box_points

def predict_newline(m, b, dx = 10, dy = 10, dr = .25, frame_shape = (500, 500)):
    bx_pnts = project_viewbox(dx=dx, dy=dy, dr=dr, box_points=get_boxpoints(frame_shape))
    if dr == 0:
      m = m 
      b = m * bx_pnts[0][0] + b
      b -= bx_pnts[-1][1]
    else:
      i0 = find_line_intersection(m, b, *points_to_line(bx_pnts[0], bx_pnts[-1]))
      i1 = find_line_intersection(m, b, *points_to_line(bx_pnts[0], bx_pnts[1]))

      # Project back into original coordinate space
      new_box_center = ((bx_pnts[0][0] + bx_pnts[3][0])/2, (bx_pnts[0][1] + bx_pnts[3][1])/2)
      i0 = rotate_about(i0, new_box_center, -dr)
      i1 = rotate_about(i1, new_box_center, -dr)

      i0 = (i0[0] - dx, i0[1] - dy)
      i1 = (i1[0] - dx, i1[1] - dy)

      m, b = points_to_line(i0, i1)
    
    return m, b

def draw_boxpoints(frame, box_points, color = (255, 0, 0)):
  for i in range(4):
    cv2.line(frame, [int(i) for i in box_points[i]], [int(i) for i in box_points[i-1]], color, 3)
  return frame

def draw_line(frame, slope, bias, color = (0, 255, 0), thickness = 2):
  if slope > 0:
    cv2.line(frame, (0, int(bias)), (int((frame.shape[0] - bias)/slope), frame.shape[0]), color, thickness)
  else:
    if slope != 0:
      xi = int(-bias/slope) 
    else:
      xi = 0 
    cv2.line(frame, (0, int(bias)), (xi, 0), color, thickness)
  return frame

if __name__ == "__main__":
  import cv2

  true_frame_hw = (250, 500) 
  view_area = (true_frame_hw[0] * 5, true_frame_hw[1] * 2, 3)
  frame = np.zeros(view_area)

  dx = 0
  dy = 15
  dr = np.deg2rad(0)
  m, b = 3, 25

  frame = draw_line(frame, m, b)
  bx_pnts = get_boxpoints(true_frame_hw)
  frame = draw_boxpoints(frame, bx_pnts)

  for _ in range(1):
    bx_pnts = project_viewbox(dx=dx, dy=dy, dr=dr, box_points=bx_pnts)
    frame = draw_boxpoints(frame, bx_pnts)

    if dr == 0:
      m = m 
      b = m * bx_pnts[0][0] + b
      b -= bx_pnts[-1][1]
    else:
      frame = draw_line(frame, *points_to_line(bx_pnts[0], bx_pnts[-1]), color=(0,0,255))
      frame = draw_line(frame, *points_to_line(bx_pnts[0], bx_pnts[1]), color=(0,0,255))

      i0 = find_line_intersection(m, b, *points_to_line(bx_pnts[0], bx_pnts[-1]))
      i1 = find_line_intersection(m, b, *points_to_line(bx_pnts[0], bx_pnts[1]))

      cv2.circle(frame, [int(i) for i in i0], 12, (255,0,255), -1)
      cv2.circle(frame, [int(i) for i in i1], 12, (255,0,255), -1)

      # Project back into original coordinate space
      new_box_center = ((bx_pnts[0][0] + bx_pnts[3][0])/2, (bx_pnts[0][1] + bx_pnts[3][1])/2)
      i0 = rotate_about(i0, new_box_center, -dr)
      i1 = rotate_about(i1, new_box_center, -dr)

      i0 = (i0[0] - dx, i0[1] - dy)
      i1 = (i1[0] - dx, i1[1] - dy)

      cv2.circle(frame, [int(i) for i in i0], 12, (60,125,205), -1)
      cv2.circle(frame, [int(i) for i in i1], 12, (60,125,205), -1)

      m, b = points_to_line(i0, i1)
    frame = draw_line(frame, m, b, color=(0,255,255))

  cv2.imshow("frame", frame)
  cv2.waitKey(0)