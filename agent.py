"""
agent template for the nav challenge
"""
class Agent:
  def __init__(self, cfg:dict):
    """cfg keys: width_m, height_m, resolution, robot_radius, v_max, a_max, dt, sense_cells, goal_tol, goal (x, y)."""
    self.cfg = cfg
    self.movement = Movement(cfg)
  def step(self, pose:tuple[float, float], scan:tuple[int, int, list[str]]) -> tuple[float, float]:




    """
    called once per tick.

    pose: (x, y) metres from SLAM, ~2 cm gaussian noise.
    scan: (cx0, cy0, rows) -- a (2*sense_cells+1)^2 window of '#'/'.' around the robot. rows[j][i] is cell (cx0+i, cy0+j).
          everything in the window is observed, nothing outside it is.
          the window origin comes from the noisy pose, so walls can land one cell off between scans.
    returns: (vx, vy) world-frame velocity command in m/s. sim clamps speed and acceleration.
    """

    return self.movement.move(pose)

  def debug(self) -> dict:
    """
    optional, for `harness.py --viz` only.
    keys: blocked (cells), free (cells), path ([(x, y), ...]).
    """
    return {}

class Movement:

  def __init__(self, cfg):
    self.cfg = cfg

  def move(self, pose):
    x, y = pose
    gx, gy = self.cfg["goal"]

    dx = gx - x
    dy = gy - y
    distance = (dx * dx + dy * dy) ** 0.5

    if distance < self.cfg["goal_tol"]:
      return (0, 0)

    vx = dx / distance
    vy = dy / distance

    return (vx * self.cfg["v_max"], vy * self.cfg["v_max"])
