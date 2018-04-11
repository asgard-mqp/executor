
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Polygon

from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg

from world import World

world = World()

def to_point(pos):
  x, y = pos
  x, y = (x*0.6096, y*0.6096)
  return Point32(x, y, 0)

def to_square(point, radius):
  x, y = point
  points = [
    to_point((x + radius, y + radius)),
    to_point((x + radius, y - radius)),
    to_point((x - radius, y - radius)),
    to_point((x - radius, y + radius)),
  ]

  return Polygon(points)

def to_obsarray_bulked():
  array = ObstacleArrayMsg()
  array.header.frame_id = 'field'

  oid = [0]
  def next_id():
    oid[0] += 1
    return oid[0] - 1

  lines = [
    # small cone corner
    ((0, 0), (0, 2.125)),
    ((0, 0), (2.125, 0)),
    ((1, 0), (1, 2.125)),
    ((0, 1), (2.125, 1)),

    # long cone corner
    ((6, 6), (6, 6 - 3.125)),
    ((6, 6), (6 - 3.125, 6)),
    ((5, 6), (5, 6 - 3.125)),
    ((6, 5), (6 - 3.125, 5)),

    ( # center cones
      (2, 2), (2, 2.5),
      (3.5, 4), (4, 4),
      (4, 3.5), (2.5, 2)
    ),

    # goal line
    ((0, 4.1), (1.9, 6)),
    ((4.1, 0), (6, 1.9))
  ]

  for l in lines:
    obs = ObstacleMsg(polygon=Polygon([to_point(p) for p in l]))
    obs.id = next_id()
    array.obstacles.append(obs)

  for stationary in ((2, 4), (4, 2)):
    obs = ObstacleMsg(polygon=to_square(stationary, 0.127))
    obs.id = next_id()
    array.obstacles.append(obs)

  return array

def to_obsarray():
  array = ObstacleArrayMsg()
  array.header.frame_id = 'field'

  oid = [0]
  def next_id():
    oid[0] += 1
    return oid[0] - 1

  red_line  = [to_point((0, 4.1)), to_point((1.9, 6))]
  blue_line = [to_point((4.1, 0)), to_point((6, 1.9))]

  for l in (blue_line, red_line):
    obs = ObstacleMsg(polygon=Polygon(l))
    obs.id = next_id()
    array.obstacles.append(obs)

  for stationary in ((2, 4), (4, 2)):
    obs = ObstacleMsg(polygon=to_square(stationary, 0.127))
    obs.id = next_id()
    array.obstacles.append(obs)

  for color, pos in world.goals:
    obs = ObstacleMsg(polygon=to_square(pos, 0.127))
    obs.id = next_id()
    array.obstacles.append(obs)

  for pos in world.cones:
    obs = ObstacleMsg(polygon=to_square(pos, 0.076))
    obs.id = next_id()
    array.obstacles.append(obs)

  return array

def to_pointcloud():
  pcl = PointCloud()
  pcl.header.frame_id = 'field'

  for color, pos in world.goals:
    pcl.points.append(to_point(pos))
  
  # for pos in world.cones:
  #   pcl.points.append(to_point(pos))

  return pcl

if __name__ == '__main__':
  rospy.init_node('obstacles')
  # points = rospy.Publisher('/asgard/obstacles', PointCloud, queue_size=1)
  obstacles = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  
  rate = rospy.Rate(0.5)
  while not rospy.is_shutdown():
    # points.publish(to_pointcloud())
    # obstacles.publish(to_obsarray())
    obstacles.publish(to_obsarray_bulked())
    rate.sleep()
  