import numpy as np

def find_line_itersection(line1, line2):
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

def points_to_line(p1, p2):
  # when given two points, return a slope and bias value which will intersect with those given points
  x1, y1 = p1
  x2, y2 = p2
  m = (y2 - y1) / (x2 - x1)
  b = y1 - m * x1     
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
  return [(0, box_hw[1]), box_hw, (box_hw[0],0), (0,0)]

def project_viewbox(dx = 10, dy = 10, dr = .25, box_points = [(0, 1), (1,1), (1,0), (0,0)]):
  box_center = ((box_points[0][0] + box_points[3][0])/2, (box_points[0][1] + box_points[3][1])/2)
  
  box_points = list(map(lambda pt: rotate_about(pt, box_center, dr), box_points))
  
  box_points = list(map(lambda pt: (pt[0] + dx, pt[1] + dy), box_points))
  return box_points

def predict_newline(m, b, dx = 10, dy = 10, dr = .25, frame_shape = (500, 500)):
  # Get points on box corners
  bx_pnts = get_boxpoints(frame_shape)
  bx_pnts = project_viewbox(dx = dx, dy = dy, dr = dr, box_points = bx_pnts)
  i0 = find_line_itersection([*line_to_points(m, b)], (bx_pnts[2], bx_pnts[3]))
  i1 = find_line_itersection([*line_to_points(m, b)], (bx_pnts[0], bx_pnts[1]))

  # Project back into original coordinate space
  i0 = (i0[0] - dx, i0[1] - dy)
  i1 = (i1[0] - dx, i1[1] - dy)
  i0 = rotate_about(i0, (0, 0), -dr)
  i1 = rotate_about(i1, (0, 0), -dr)

  # Convert to new slope and bias
  return points_to_line(i0, i1)

true_frame_shape = (250, 500)

dx = 1
dy = 1
dr = 0

m, b = 0, .5
# Convert bias to pixel value
b = b * true_frame_shape[0]

for _ in range(15):
  try:
    m, b = predict_newline(m, b, dx, dy, dr, true_frame_shape)
    print(round(m,3), round(b,3))
  except:
    print(2)