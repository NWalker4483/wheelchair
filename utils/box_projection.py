import numpy as np
import cv2
from .draw import draw_line, draw_boxpoints, get_boxpoints
from .math import points_to_line, line_to_points, rotate_about, find_line_intersection

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

if __name__ == "__main__":
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