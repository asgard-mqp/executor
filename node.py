import rospy, actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import PoseStamped, Twist

from utils import quat_from_text, yaw_from_text, mt, in_front

actions = []
current_action = None
current_expected_location = (0, 0, yaw_from_text('upright'))

nav = None
arm_up = None
twist = None

def arm_state(self):
  pass

def to_pose(x, y, quat):
  # 2 ft -> meters
  x, y = (x*0.6096, y*0.6096)
  new_pose = PoseStamped()
  new_pose.pose.position.x = x
  new_pose.pose.position.y = y
  new_pose.pose.orientation = quat
  new_pose.header.frame_id = 'field'
  return new_pose

def register():
  global nav, arm_up, twist
  rospy.Subscriber("/asgard/action", String, new_action)
  rospy.Subscriber("/arm_state", Int16, arm_state)
  nav = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
  arm_up = rospy.Publisher("/arm_goal", Bool, queue_size=1)
  twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

def new_action(data):
  # i'm too lazy to make this a custom type
  try:
    data = data.data
  except AttributeError:
    pass
  
  message = data.split(" ")[0]
  args = data.split(" ")[1:]

  func = {
    "GOAL": pickup_goal,
    # "CONE": pickup_cone,
    "SCORE": score_goal,
    "POS": drive_to
  }[message]

  actions.append((func, args))

  if not current_action:
    next_action()

def next_action():
  global current_action
  print "completed: ", current_action
  rospy.sleep(0.25)

  if actions:
    current_action = actions.pop(0)
    print "=> ", current_action, " // ", actions
    current_action[0](*current_action[1])
  else:
    current_action = None

def drive_to(x, y, direction, callback=None):
  x, y = float(x), float(y)
  nav_pose = MoveBaseGoal()
  nav_pose.target_pose = to_pose(x, y, quat_from_text(direction))

  def nav_complete(state, result):
    global current_expected_location
    current_expected_location = (x, y, yaw_from_text(direction))
    if callback:
      callback()
    else:
      next_action()

  nav.send_goal(nav_pose, done_cb=nav_complete)

def pickup_goal():
  x, y, yaw = in_front(current_expected_location, 1.25)

  subactions = [
    (move_arm_down, mt),
    # (push_velocity, (0.5,)),
    (drive_to, (x, y, yaw)),
    (move_arm_up, mt)
  ]

  for i in subactions[::-1]:
    actions.insert(0, i)

  next_action()

goals = [
  (0.75, 6 - 0.75),
  (1,   6 - 2),
  (1.5, 6 - 1.5),
  (2,   6 - 1),

  (6 - 0.75, 0.75),
  (6 - 2,   1),
  (6 - 1.5, 1.5),
  (6 - 1,   2),
]

def score_goal(index):
  index = int(index)
  heading = 'DOWNLEFT' if index < 4 else 'UPRIGHT'
  x, y = goals[index]
  backup = in_front((x, y, yaw_from_text(heading)), -0.75)

  subactions = []
  if index == 0 or index == 4:
    staging_x, staging_y = goals[index + 2]
    backup = in_front((staging_x, staging_y, yaw_from_text(heading)), -0.75)

    subactions += [
      (drive_to, (staging_x, staging_y, heading)),
      (push_velocity, (0.4,)),
      # (drive_to, (x, y, heading)),
      (move_arm_down, mt),
      (push_velocity, (-0.4,)),
      #(drive_to, backup),
      (move_arm_up, mt)
    ]

  else:
    subactions += [
      (drive_to, (x, y, heading)),
      (push_velocity, (0.25, 1)),
      (move_arm_down, mt),
      (push_velocity, (-0.25, 1)),
      (move_arm_up, mt)
    ]

  for i in subactions[::-1]:
    actions.insert(0, i)

  next_action()

def push_velocity(vel=0.5, dur=1.5):
  fw = Twist()
  fw.linear.x = vel
  twist.publish(fw)
  # twist.publish(fw)

  rospy.sleep(dur)
  twist.publish(Twist())
  next_action()

def move_arm_down():
  arm_up.publish(Bool(False))
  rospy.sleep(0.5)
  next_action()

def move_arm_up():
  arm_up.publish(Bool(True))
  rospy.sleep(0.5)
  next_action()

def start():
  # raw_input("press enter to continue ")
  with open('plan') as f:
    for line in f:
      line = line.strip()
      if line.startswith("#"): continue
      if line: new_action(line)

if __name__ == '__main__':
  rospy.init_node('executor')
  print 1
  register()
  print 2
  #drive_to(0.5, 4.5, "UPRIGHT", callback=start)
  start()
  print 3
  rospy.spin()
