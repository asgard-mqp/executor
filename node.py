#!/usr/bin/env python3

import rospy, actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

from util import quat_from_text, yaw_from_text, mt
from world import World

world = World()
actions = []
current_action = None
current_expected_location = (0, 0, yaw_from_text('upright'))

nav = None
arm_down = None

def to_pose(self, x, y):
  # 2 ft -> meters
  x, y = (x*0.6096, y*0.6096)
  new_pose = PoseStamped()
  new_pose.pose.position.x = x
  new_pose.pose.position.y = y
  new_pose.header.frame_id = self.frame_id
  return new_pose

def register():
  global nav, arm_down
  rospy.Subscriber("/asgard/action", String, new_action)
  rospy.Subscriber("/asgard/arm_state", String, arm_state)
  nav = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
  arm_down = rospy.Publisher("/asgard/arm_down", Bool, queue_size=1)

def new_action(data):
  # i'm too lazy to make this a custom type
  message, *args = data.data.split(" ")
  func = {
    "GOAL": pickup_goal,
    "CONE": pickup_cone,
    "SCORE": score_goal,
    "POS": drive_to
  }[message]

  actions.append((func, args))

  if not current_action:
    next_action()

def next_action():
  global current_action
  rospy.sleep(500)

  if actions:
    current_action = actions.pop(0)
    current_action[0](*current_action[1])
  else:
    current_action = None

def drive_to(x, y, direction):
  x, y = float(x), float(y)
  nav_pose = MoveBaseGoal()
  nav_pose.target_pose = to_pose(x, y, quat_from_text(direction))

  def nav_complete():
    global current_expected_location
    current_expected_location = (x, y, yaw_from_text(direction))
    next_action()

  nav.send_goal(nav_pose)

def pickup_goal():
  x, y, yaw = in_front(current_expected_location, 0.5)

  subactions = [
    (move_arm_down, mt),
    (drive_to, (x, y, yaw)),
    (move_arm_up, mt)
  ]

  for i in subactions[::-1]:
    actions.insert(0, i)

  next_action()

goal = ['red_deep', 'red', 'red', 'red', 'blue_deep', 'blue', 'blue', 'blue']
def score_goal():
  world.closest_goal()
  subactions = [
    (move_arm_down, mt),
    (drive_to, (x, y, yaw)),
    (move_arm_up, mt)
  ]

  for i in subactions[::-1]:
    actions.insert(0, i)

def move_arm_down():
  arm_down.publish(Bool(True))
  rospy.sleep(5000)
  next_action()

def move_arm_up():
  arm_down.publish(Bool(False))
  rospy.sleep(5000)
  next_action()

if __name__ == '__main__':
  rospy.init_node('executor')
  register()
  rospy.spin()
