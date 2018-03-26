
from math import sin, cos, degrees

mt = (,)

def yaw_from_text(heading):
  heading = heading.lower().strip()
  return {
    'up': 90,
    'down': 270,
    'left': 180,
    'right': 0,
    'upleft': 45 + 90,
    'upright': 0 + 45,
    'downleft': 180 + 45
    'downright': 270 + 45
  }[heading]

def quat_from_text(heading):
  angle = yaw_from_text(heading)
  return Quaternion(*quaternion_from_euler(0, 0, angle))

def in_front(pose, distance):
  x, y, yaw = pose
  tyaw = radians(yaw)
  return (x + distance * cos(tyaw), y + distance * sin(tyaw), yaw)
