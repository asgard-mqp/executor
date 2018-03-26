import collections

goal = collections.namedtuple('goal', 'owner pos')

def with_inverse(*lst):
  return list(lst) + [(144-x, 144-y) for x, y in lst]

def to_field(lst):
  return ((x/24.0, y/24.0) for x, y in lst)

def sqdist(pos1, pos2):
  return (pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2

## coordinate system is across -> down
## red far zone is (0, 6)

class World(object):
  def __init__(self):
    self.goals = set((
      goal('red', (0.5, 1.5)),
      goal('red', (3., 2.)),
      goal('red', (4., 3.)),
      goal('red', (4.5, 5.5)),

      goal('blue', (1.5, 0.5)),
      goal('blue', (2., 3.)),
      goal('blue', (3., 4.)),
      goal('blue', (5.5, 4.5))
    ))

    self.scoring_locations = [
      goal('red', (0.75, 6 - 0.75)),
      goal('red', (1.75, 6 - 0.75)),
      goal('red', (1.25, 6 - 1.25)),
      goal('red', (0.75, 6 - 1.75)),

      goal('blue', (6 - 0.75, 0.75)),
      goal('blue', (6 - 1.75, 0.75)),
      goal('blue', (6 - 1,    1)),
      goal('blue', (6 - 0.5,  1.75)),
    ]

    self.cones = set(to_field(with_inverse(
      (3,    3),
      (13.5, 3),
      (24,   3),
      (3,    13.5),
      (13.5, 13.5),
      (24,   13.5),
      (3,    24),
      (13.5, 24),
      (24,   24),
      (3,    36),
      (24,   36),
      (3,    48),
      (24,   48),
      (36,   3),
      (36,   24),
      (48,   3),
      (48,   24),

      (48, 48),
      (48, 60),
      (60, 48),
      (72, 60),
      (60, 72),
    ) + [
      (144-3,  144-60),
      (144-24, 144-60),
      (144-3,  144-72),
      (144-24, 144-72),
      (144-60, 144-3),
      (144-60, 144-24),
      (144-72, 144-3),
      (144-72, 144-24),
    ]))

  def goals_for(self, color):
    return filter(lambda g: g.owner == color, self.goals)

  def closest_goal(self, color, loc):
    return min(self.goals_for(color), key=lambda g: sqdist(g.pos, loc))
