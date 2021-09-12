import cv2

def get_boxpoints(box_hw):
  return [(0, box_hw[0]), box_hw[::-1], (box_hw[1], 0), (0,0)]

def draw_boxpoints(frame, box_points, color = (255, 0, 0)):
  for i in range(4):
    cv2.line(frame, [int(i) for i in box_points[i]], [int(i) for i in box_points[i-1]], color, 3)
  return frame
  
def draw_line(frame, slope, bias, color = (0, 255, 0), thickness = 2, swap_xy = False):
  if swap_xy:
    p1 = (int(bias), 0)
    p2 = (int(slope * frame.shape[0] + bias), frame.shape[0])
  else:
    p1 = (0, int(bias))
    p2 = (frame.shape[1], int(slope * frame.shape[1] + bias))

  cv2.line(frame, p1, p2, color, thickness)
  return frame

