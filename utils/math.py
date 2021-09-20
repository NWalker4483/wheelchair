
import numpy as np

def sign(a):
  """ ... """
  return 1 if a > 0 else -1
    
def unit_vector(vector):
  """ Returns the unit vector of the vector.  """
  return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
        >>> angle_between((1, 0, 0), (0, 1, 0))
        1.5707963267948966
        >>> angle_between((1, 0, 0), (1, 0, 0))
        0.0
        >>> angle_between((1, 0, 0), (-1, 0, 0))
        3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def distance(p1,p2):
  return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**.5

def midpoint(p1,p2):
  x1,y1 = p1
  x2,y2 = p2
  x_m_point = (x1 + x2)/2
  y_m_point = (y1 + y2)/2
  return (x_m_point, y_m_point)

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
  """ when given two points, 
  returns a slope and bias value which will intersect with those given points
  """
  x1, y1 = p1
  x2, y2 = p2
  m = (y2 - y1) / (x2 - x1)
  b = y1 - (m * x1)
  return m, b

def line_to_points(m, b):
  # Returns origin and unit vector
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

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)
    
def turn_clockwise(v1, v2):
  # Determine whether the shortest rotation between two vector is Counterclockwise or clockwise
  d1 = rotate_about((0,1),(0,0),v2 + np.deg2rad(90))
  d2 = rotate_about((0,1),(0,0),v2 - np.deg2rad(90))
            
  return (angle_between(v1,d1) > angle_between(v1,d2)).all()
