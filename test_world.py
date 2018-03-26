
from world import World, goal

def test_closest_goal():
    w = World()
    assert w.closest_goal('blue', (0, 0)) == goal('blue', (1.5, 0.5))
    assert w.closest_goal('red', (0, 0)) == goal('red', (0.5, 1.5))
