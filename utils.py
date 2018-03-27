
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from math import sin, cos, degrees, radians

mt = tuple()

def yaw_from_text(heading):
  try:
    heading = heading.lower().strip()
  except AttributeError:
    return heading

  return {
    'up': -90,
    'down': 90,
    'left': 180,
    'right': 0,
    'upleft': -(90 + 45),
    'upright': -45,
    'downleft': 90 + 45,
    'downright': 45
  }[heading]

def quat_from_text(heading):
  angle = yaw_from_text(heading)
  return Quaternion(*quaternion_from_euler(0, 0, radians(angle)))

def in_front(pose, distance):
  x, y, yaw = pose
  tyaw = radians(yaw)
  return (x + distance * cos(tyaw), y + distance * sin(tyaw), yaw)
