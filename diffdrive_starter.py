
import math
import sys
import pygame

# ----------------------------------------------------------------------------
# Parameters  (Part 1.3: you will vary these)
# ----------------------------------------------------------------------------
WIDTH, HEIGHT = 900, 650
SCALE = 100.0                          # pixels per metre  (100 px = 1 m)
FPS = 60
DT = 1.0 / FPS                         # simulation time step [s]
ORIGIN = (WIDTH // 2, HEIGHT // 2)    # pixel position of world (0, 0)

WHEEL_RADIUS = 0.05                    # r  [m]
WHEEL_BASE = 0.30                      # L  [m]  distance between the two wheels
W_MAX = 12.0                           # maximum wheel angular speed [rad/s]
START_POSE = (-3.0, -2.0, 0.0)         # (x [m], y [m], theta [rad])

K_RHO = 1.0                            # go-to-goal gain on distance (Part 1.2)
K_ALPHA = 3.0                          # go-to-goal gain on heading error (Part 1.2)
GOAL_TOLERANCE = 0.05                  # stop when distance to goal < this [m]
CIRCLE_RADIUS = 1.0                    # R for circle mode [m]  (Part 1.1)
CIRCLE_SPEED = 0.4                     # forward speed v for circle mode [m/s]


# ----------------------------------------------------------------------------
# Coordinate helpers (working)
# ----------------------------------------------------------------------------
def world_to_screen(x, y):
    """Metres (y up) -> pixels (y down)."""
    return int(ORIGIN[0] + x * SCALE), int(ORIGIN[1] - y * SCALE)


def screen_to_world(px, py):
    """Pixels -> metres. Used to turn a mouse click into goal B."""
    return (px - ORIGIN[0]) / SCALE, (ORIGIN[1] - py) / SCALE


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, lo, hi):
    """Clamp value to range [lo, hi]."""
    return max(lo, min(hi, value))


# ----------------------------------------------------------------------------
# Kinematics
# ----------------------------------------------------------------------------
def forward_kinematics(w_left, w_right):
    """Forward Kinematics (FK): wheel angular speeds -> body velocities.
         v     = (r / 2) * (wR + wL)     forward linear speed [m/s]
         omega = (r / L) * (wR - wL)     yaw rate (CCW +)     [rad/s]
    Line mapping in report: Equations (1) & (2).
    """
    v = (WHEEL_RADIUS / 2.0) * (w_right + w_left)
    omega = (WHEEL_RADIUS / WHEEL_BASE) * (w_right - w_left)
    return v, omega


def inverse_kinematics(v, omega, saturate=True):
    """Inverse Kinematics (IK) (Part 1.2): desired (v, omega) -> wheel speeds (wL, wR).

    Derived by inverting FK:
         wR = (2v + omega * L) / (2r) = v/r + (omega * L) / (2r)
         wL = (2v - omega * L) / (2r) = v/r - (omega * L) / (2r)

    Saturates each wheel to +/- W_MAX.
    Note on saturation (Part 1.3 / Part 4.4):
    If wR and wL are clamped independently, their ratio wR/wL changes,
    altering the instantaneous radius of curvature R = v/omega and distorting the path.
    """
    w_right = (2.0 * v + omega * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    w_left  = (2.0 * v - omega * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)

    if saturate:
        w_left  = clamp(w_left, -W_MAX, W_MAX)
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
        self.elapsed_time = 0.0
        self.integration_mode = "EULER"  # or "EXACT_ARC" for Part 4.2 comparison

    def set_wheels(self, w_left, w_right):
        self.w_left = clamp(w_left, -W_MAX, W_MAX)
        self.w_right = clamp(w_right, -W_MAX, W_MAX)

    def step(self, dt):
        """Pose update:
        Standard mode: Forward-Euler integration of
             x     <- x     + v * cos(theta) * dt
             y     <- y     + v * sin(theta) * dt
             theta <- theta + omega * dt

        Exact-Arc mode (Part 4.2): Closed-form integration along circular arc:
             x     <- x + (v/omega) * [sin(theta + omega*dt) - sin(theta)]
             y     <- y - (v/omega) * [cos(theta + omega*dt) - cos(theta)]
             theta <- theta + omega * dt
        """
        self.v, self.omega = forward_kinematics(self.w_left, self.w_right)

        if self.integration_mode == "EULER" or abs(self.omega) < 1e-9:
            self.x += self.v * math.cos(self.theta) * dt
            self.y += self.v * math.sin(self.theta) * dt
        else:
            # Exact circular arc integration
            self.x += (self.v / self.omega) * (math.sin(self.theta + self.omega * dt) - math.sin(self.theta))
            self.y -= (self.v / self.omega) * (math.cos(self.theta + self.omega * dt) - math.cos(self.theta))

        self.theta = wrap_angle(self.theta + self.omega * dt)
        self.elapsed_time += dt


# ----------------------------------------------------------------------------
# Motion behaviours
# ----------------------------------------------------------------------------
def keyboard_command(keys):
    """Manual control (working): arrow keys -> wheel speeds."""
    forward = (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * W_MAX * 0.5
    turn = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * W_MAX * 0.25
    return forward + turn, forward - turn     # (w_left, w_right)


def go_to_goal(robot, goal, improved=False):
    """Autonomous A -> B (Part 1.2). Returns (w_left, w_right).

    Control logic:
      1. rho   = distance from robot to goal = sqrt((xg - x)^2 + (yg - y)^2)
      2. alpha = wrap_angle(atan2(yg - y, xg - x) - theta)  [heading error]
      3. if rho < GOAL_TOLERANCE: stop (return 0.0, 0.0)
      4. Linear speed v = K_RHO * rho  (capped by motor limits)
         Angular speed omega = K_ALPHA * alpha
      5. Improved mode (Part 1.3 / Part 4.5):
         When goal is behind robot (|alpha| > pi/2), pure forward drive causes wide ballooning loops.
         Modulate v with cos(alpha): v = K_RHO * rho * max(0.0, cos(alpha)).
         This smoothly slows forward translation and prioritizes turning when misaligned!
      6. Return inverse_kinematics(v, omega).
    """
    gx, gy = goal
    dx = gx - robot.x
    dy = gy - robot.y
    rho = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - robot.theta)

    if rho < GOAL_TOLERANCE:
        return 0.0, 0.0

    # Max achievable linear speed from max wheel angular velocity
    v_max = WHEEL_RADIUS * W_MAX

    if improved:
        # Cosine-modulated velocity for smooth turning when heading error is large
        v = K_RHO * rho * max(0.0, math.cos(alpha))
    else:
        v = K_RHO * rho

    v = min(v, v_max)
    omega = K_ALPHA * alpha

    return inverse_kinematics(v, omega)


def circle_command(radius, speed):
    """Constant-arc motion (Part 1.1 case iv). Returns (w_left, w_right).

    For a circle of radius R at linear speed v:
         omega = v / R
    Inverse kinematics translates (v, omega) directly into wheel speeds:
         wR = v/r + (omega * L) / (2r)
         wL = v/r - (omega * L) / (2r)
    """
    omega = speed / radius
    return inverse_kinematics(speed, omega)


def figure_eight_command(t, a=1.0, speed=0.4):
    """Part 3 Live demo bonus: Figure-eight (Lemniscate) trajectory controller."""
    # Alternates turning left and right with period T_period
    period = 8.0
    phase = (t % period) / period
    omega = 0.5 * math.sin(2.0 * math.pi * phase)
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
START_COLOR = (220, 120, 20)
TRAIL_COLOR = (80, 110, 210)


def to_screen_pts(robot, local_pts):
    """Rotate/translate body-frame points (metres) into pixel coordinates."""
    c, s = math.cos(robot.theta), math.sin(robot.theta)
    return [world_to_screen(robot.x + c * px - s * py,
                            robot.y + s * px + c * py) for px, py in local_pts]


def draw_grid(screen):
    for k in range(-5, 6):                       # 1 m grid lines
        x0, _ = world_to_screen(k, 0)
        _, y0 = world_to_screen(0, k)
        pygame.draw.line(screen, GRID, (x0, 0), (x0, HEIGHT))
        pygame.draw.line(screen, GRID, (0, y0), (WIDTH, y0))
    pygame.draw.circle(screen, (180, 180, 180), ORIGIN, 4)


def draw_robot(screen, robot):
    half_l, half_w = 0.20, WHEEL_BASE / 2.0
    body = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    pygame.draw.polygon(screen, BODY, to_screen_pts(robot, body))
    for side in (+1, -1):                        # the two driven wheels
        w = [(0.06, side * half_w), (-0.06, side * half_w),
             (-0.06, side * (half_w + 0.03)), (0.06, side * (half_w + 0.03))]
        pygame.draw.polygon(screen, WHEEL, to_screen_pts(robot, w))
    nose = to_screen_pts(robot, [(0, 0), (half_l + 0.10, 0)])
    pygame.draw.line(screen, HEAD, nose[0], nose[1], 3)


def draw_trail(screen, trail):
    """Part 1.1: Draw trajectory trail of the robot."""
    if len(trail) >= 2:
        pixel_points = [world_to_screen(x, y) for x, y in trail]
        pygame.draw.lines(screen, TRAIL_COLOR, False, pixel_points, 2)


def draw_start_and_goal(screen, start_pose, goal):
    """Draw Start (A) and Goal (B) markers with clear labels."""
    font = pygame.font.SysFont("consolas,menlo,monospace", 14, bold=True)
    # Start Marker
    sx, sy = world_to_screen(start_pose[0], start_pose[1])
    pygame.draw.circle(screen, START_COLOR, (sx, sy), 6)
    screen.blit(font.render("A (Start)", True, START_COLOR), (sx + 8, sy - 8))

    # Goal Marker
    if goal is not None:
        gx, gy = world_to_screen(*goal)
        pygame.draw.circle(screen, GOAL, (gx, gy), 8, 2)
        pygame.draw.line(screen, GOAL, (gx - 10, gy), (gx + 10, gy), 1)
        pygame.draw.line(screen, GOAL, (gx, gy - 10), (gx, gy + 10), 1)
        screen.blit(font.render(f"B ({goal[0]:.2f}, {goal[1]:.2f})", True, GOAL), (gx + 10, gy - 10))


def draw_hud(screen, font, robot, mode, goal, improved_ctrl):
    """Part 1.1: Comprehensive on-screen telemetry display."""
    theta_deg = math.degrees(robot.theta)
    lines = [
        f"Mode: {mode}  |  Integration: {robot.integration_mode}  |  Ctrl: {'Improved (Cos)' if improved_ctrl else 'Standard'}",
        f"Pose: x = {robot.x:6.2f} m,  y = {robot.y:6.2f} m,  theta = {theta_deg:6.1f} deg",
        f"Body: v = {robot.v:6.3f} m/s,  omega = {robot.omega:6.3f} rad/s",
        f"Motors: wL = {robot.w_left:6.2f} rad/s,  wR = {robot.w_right:6.2f} rad/s  (W_MAX = {W_MAX:.1f})",
        f"Sim Time: t = {robot.elapsed_time:6.2f} s",
    ]
    if goal is not None and mode == "GO_TO_GOAL":
        dist = math.hypot(goal[0] - robot.x, goal[1] - robot.y)
        lines.append(f"Target B: ({goal[0]:.2f}, {goal[1]:.2f}) m  |  Distance: {dist:5.2f} m")

    # Render translucent background for HUD
    hud_surf = pygame.Surface((560, 24 + 20 * len(lines)))
    hud_surf.set_alpha(200)
    hud_surf.fill((255, 255, 255))
    screen.blit(hud_surf, (8, 6))

    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, (20, 20, 20)), (14, 10 + 20 * i))

    # Controls help in bottom-left corner
    help_lines = [
        "Keys: 1=Keyboard | 2=Go-to-Goal (Click to set B) | 3=Circle | 4=Fig-8",
        "R=Reset | C=Clear Trail | I=Toggle Euler/Arc | M=Toggle Ctrl Mode | ESC=Quit"
    ]
    small_font = pygame.font.SysFont("consolas,menlo,monospace", 13)
    for i, htext in enumerate(help_lines):
        screen.blit(small_font.render(htext, True, (80, 80, 80)), (14, HEIGHT - 45 + 18 * i))


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("CS2110 - Differential Drive Kinematics Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas,menlo,monospace", 15)

    robot = Robot(START_POSE)
    mode = "KEYBOARD"
    goal = (0.0, 0.0)             # Default goal point B
    trail = [(robot.x, robot.y)]
    improved_ctrl = False

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
                elif event.key == pygame.K_4:
                    mode = "FIGURE_EIGHT"
                elif event.key == pygame.K_r:
                    robot.reset(START_POSE)
                    trail.clear()
                    trail.append((robot.x, robot.y))
                elif event.key == pygame.K_c:
                    trail.clear()
                    trail.append((robot.x, robot.y))
                elif event.key == pygame.K_i:
                    robot.integration_mode = "EXACT_ARC" if robot.integration_mode == "EULER" else "EULER"
                elif event.key == pygame.K_m:
                    improved_ctrl = not improved_ctrl
            elif event.type == pygame.MOUSEBUTTONDOWN:
                goal = screen_to_world(*event.pos)       # Click to set goal B
                if mode != "GO_TO_GOAL":
                    mode = "GO_TO_GOAL"

        # ---- choose wheel speeds according to the active mode ----
        if mode == "KEYBOARD":
            wl, wr = keyboard_command(pygame.key.get_pressed())
        elif mode == "GO_TO_GOAL" and goal is not None:
            wl, wr = go_to_goal(robot, goal, improved=improved_ctrl)
        elif mode == "CIRCLE":
            wl, wr = circle_command(CIRCLE_RADIUS, CIRCLE_SPEED)
        elif mode == "FIGURE_EIGHT":
            wl, wr = figure_eight_command(robot.elapsed_time)
        else:
            wl, wr = 0.0, 0.0

        robot.set_wheels(wl, wr)

        # ---- advance the simulation one step ----
        robot.step(DT)
        trail.append((robot.x, robot.y))

        # ---- draw ----
        screen.fill(BG)
        draw_grid(screen)
        draw_trail(screen, trail)
        draw_start_and_goal(screen, START_POSE, goal)
        draw_robot(screen, robot)
        draw_hud(screen, font, robot, mode, goal, improved_ctrl)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
