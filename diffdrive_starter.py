"""
CS2110 Introduction to Robotics -- CIE-3 Project
Differential Drive Robot: COMPLETED CODE

Run:  python diffdrive_starter.py        (requires: pip install pygame)
"""
import math
import importlib
import sys

try:
    pygame = importlib.import_module("pygame")
except ModuleNotFoundError as exc:
    if exc.name != "pygame":
        raise
    raise SystemExit("pygame is required; install it with: python -m pip install pygame") from exc

# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
WIDTH, HEIGHT = 900, 650
SCALE = 100.0                 # pixels per metre  (100 px = 1pip ins m)
FPS = 60
DT = 1.0 / FPS                # simulation time step [s]
ORIGIN = (WIDTH // 2, HEIGHT // 2)   # pixel position of world (0, 0)

WHEEL_RADIUS = 0.05           # r  [m]
WHEEL_BASE = 0.30             # L  [m]  distance between the two wheels
W_MAX = 12.0                  # maximum wheel angular speed [rad/s]
START_POSE = (-3.0, -2.0, 0.0)   # (x [m], y [m], theta [rad])

K_RHO = 1.0                   # go-to-goal gain on distance
K_ALPHA = 3.0                 # go-to-goal gain on heading error
GOAL_TOLERANCE = 0.05         # stop when distance to goal < this [m]
CIRCLE_RADIUS = 1.0           # R for circle mode [m]
CIRCLE_SPEED = 0.4            # forward speed v for circle mode [m/s]


# ----------------------------------------------------------------------------
# Coordinate helpers
# ----------------------------------------------------------------------------
def world_to_screen(x, y):
    """Metres (y up) -> pixels (y down)."""
    return int(ORIGIN[0] + x * SCALE), int(ORIGIN[1] - y * SCALE)


def screen_to_world(px, py):
    """Pixels -> metres.  Used to turn a mouse click into goal B."""
    return (px - ORIGIN[0]) / SCALE, (ORIGIN[1] - py) / SCALE


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ----------------------------------------------------------------------------
# Kinematics
# ----------------------------------------------------------------------------
def forward_kinematics(w_left, w_right):
    """FK: wheel angular speeds -> body velocities[cite: 2]."""
    v = WHEEL_RADIUS / 2.0 * (w_right + w_left)
    omega = WHEEL_RADIUS / WHEEL_BASE * (w_right - w_left)
    return v, omega


def inverse_kinematics(v, omega):
    """IK: desired (v, omega) -> wheel angular speeds (wL, wR)[cite: 2]."""
    r = WHEEL_RADIUS
    L = WHEEL_BASE
    w_left = (2.0 * v - omega * L) / (2.0 * r)
    w_right = (2.0 * v + omega * L) / (2.0 * r)
    
    # Saturation of wheel speeds
    w_left = clamp(w_left, -W_MAX, W_MAX)
    w_right = clamp(w_right, -W_MAX, W_MAX)
    return w_left, w_right


# ----------------------------------------------------------------------------
# Robot model
# ----------------------------------------------------------------------------
class Robot:
    def __init__(self, pose=START_POSE):
        self.reset(pose)

    def reset(self, pose=START_POSE):
        self.x, self.y, self.theta = pose
        self.w_left = 0.0
        self.w_right = 0.0
        self.v = 0.0
        self.omega = 0.0

    def set_wheels(self, w_left, w_right):
        self.w_left = clamp(w_left, -W_MAX, W_MAX)
        self.w_right = clamp(w_right, -W_MAX, W_MAX)

    def step(self, dt):
        """Pose update via forward-Euler integration[cite: 6]."""
        self.v, self.omega = forward_kinematics(self.w_left, self.w_right)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = wrap_angle(self.theta + self.omega * dt)


# ----------------------------------------------------------------------------
# Motion behaviours
# ----------------------------------------------------------------------------
def keyboard_command(keys):
    """Manual control: arrow keys -> wheel speeds[cite: 6]."""
    forward = (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * W_MAX * 0.5
    turn = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * W_MAX * 0.25
    return forward + turn, forward - turn


def go_to_goal(robot, goal):
    """Autonomous A -> B go-to-goal controller[cite: 2, 6]."""
    gx, gy = goal
    rho = math.sqrt((gx - robot.x)**2 + (gy - robot.y)**2)
    
    if rho < GOAL_TOLERANCE:
        return 0.0, 0.0
        
    alpha = wrap_angle(math.atan2(gy - robot.y, gx - robot.x) - robot.theta)
    
    # Control laws with speed saturation handling
    v = min(K_RHO * rho, 1.0)  # capped linear speed for stability
    omega = K_ALPHA * alpha
    
    return inverse_kinematics(v, omega)


def circle_command(radius, speed):
    """Constant-arc motion for a circle of radius R at forward speed v[cite: 2, 6]."""
    omega = speed / radius
    return inverse_kinematics(speed, omega)


# ----------------------------------------------------------------------------
# Drawing
# ----------------------------------------------------------------------------
BG = (245, 245, 245)
GRID = (225, 225, 225)
BODY = (40, 90, 170)
WHEEL = (30, 30, 30)
HEAD = (230, 80, 40)
GOAL = (30, 160, 70)
TRAIL_COLOR = (70, 130, 180)


def to_screen_pts(robot, local_pts):
    c, s = math.cos(robot.theta), math.sin(robot.theta)
    return [world_to_screen(robot.x + c * px - s * py,
                            robot.y + s * px + c * py) for px, py in local_pts]


def draw_grid(screen):
    for k in range(-4, 5):
        x0, _ = world_to_screen(k, 0)
        _, y0 = world_to_screen(0, k)
        pygame.draw.line(screen, GRID, (x0, 0), (x0, HEIGHT))
        pygame.draw.line(screen, GRID, (0, y0), (WIDTH, y0))
    pygame.draw.circle(screen, (180, 180, 180), ORIGIN, 4)


def draw_robot(screen, robot):
    half_l, half_w = 0.20, WHEEL_BASE / 2.0
    body = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    pygame.draw.polygon(screen, BODY, to_screen_pts(robot, body))
    for side in (+1, -1):
        w = [(0.06, side * half_w), (-0.06, side * half_w),
             (-0.06, side * (half_w + 0.03)), (0.06, side * (half_w + 0.03))]
        pygame.draw.polygon(screen, WHEEL, to_screen_pts(robot, w))
    nose = to_screen_pts(robot, [(0, 0), (half_l + 0.10, 0)])
    pygame.draw.line(screen, HEAD, nose[0], nose[1], 3)


def draw_trail(screen, trail):
    """Draw the recorded trajectory points[cite: 4, 6]."""
    if len(trail) >= 2:
        pixel_points = [world_to_screen(x, y) for x, y in trail]
        pygame.draw.lines(screen, TRAIL_COLOR, False, pixel_points, 2)


def draw_goal(screen, goal):
    if goal is not None:
        gx, gy = world_to_screen(*goal)
        pygame.draw.circle(screen, GOAL, (gx, gy), 7, 2)
        pygame.draw.line(screen, GOAL, (gx - 10, gy), (gx + 10, gy), 1)
        pygame.draw.line(screen, GOAL, (gx, gy - 10), (gx, gy + 10), 1)


def draw_hud(screen, font, robot, mode, elapsed_time):
    lines = [f"mode: {mode}   time: {elapsed_time:5.1f}s",
             f"x  = {robot.x:6.3f} m     y = {robot.y:6.3f} m     theta = {math.degrees(robot.theta):6.1f} deg",
             f"wL = {robot.w_left:6.2f} rad/s   wR = {robot.w_right:6.2f} rad/s",
             f"v  = {robot.v:6.3f} m/s     omega = {robot.omega:6.3f} rad/s"]
    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, (20, 20, 20)), (10, 8 + 20 * i))


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("CS2110 - Differential Drive Kinematics")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas,menlo,monospace", 16)

    robot = Robot()
    mode = "KEYBOARD"
    goal = None
    trail = []
    elapsed_time = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_1:
                    mode = "KEYBOARD"
                elif event.key == pygame.K_2:
                    mode = "GO_TO_GOAL"
                elif event.key == pygame.K_3:
                    mode = "CIRCLE"
                elif event.key == pygame.K_r:
                    robot.reset()
                    trail.clear()
                    elapsed_time = 0.0
            elif event.type == pygame.MOUSEBUTTONDOWN:
                goal = screen_to_world(*event.pos)
                trail.clear()
                elapsed_time = 0.0

        # ---- choose wheel speeds according to the active mode ----
        if mode == "KEYBOARD":
            wl, wr = keyboard_command(pygame.key.get_pressed())
        elif mode == "GO_TO_GOAL" and goal is not None:
            wl, wr = go_to_goal(robot, goal)
        elif mode == "CIRCLE":
            wl, wr = circle_command(CIRCLE_RADIUS, CIRCLE_SPEED)
        else:
            wl, wr = 0.0, 0.0
        robot.set_wheels(wl, wr)

        # ---- advance simulation step ----
        robot.step(DT)
        elapsed_time += DT

        # Append position to trail periodically/every frame
        if len(trail) == 0 or math.hypot(robot.x - trail[-1][0], robot.y - trail[-1][1]) > 0.02:
            trail.append((robot.x, robot.y))

        # ---- draw ----
        screen.fill(BG)
        draw_grid(screen)
        draw_trail(screen, trail)
        draw_goal(screen, goal)
        draw_robot(screen, robot)
        draw_hud(screen, font, robot, mode, elapsed_time)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()