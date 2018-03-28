
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

from world import World

world = World()

def to_point(pos):
  x, y = pos
  x, y = (x*0.6096, y*0.6096)
  return Point32(x, y, 0)

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
  points = rospy.Publisher('/asgard/obstacles', PointCloud, queue_size=1)
  
  rate = rospy.Rate(0.5)
  while not rospy.is_shutdown():
    points.publish(to_pointcloud())
    rate.sleep()
  