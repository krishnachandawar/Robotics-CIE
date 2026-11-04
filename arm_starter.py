"""
CS2110 Introduction to Robotics -- CIE-3 Project
Planar 2R Manipulator Arm: COMPLETED CODE

Run:  python arm_starter.py              (requires: pip install pygame)
"""
import math
import sys
from importlib import import_module

try:
    pygame = import_module("pygame")
except ModuleNotFoundError:
    pygame = None

# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
WIDTH, HEIGHT = 900, 650
SCALE = 100.0                          # pixels per metre
FPS = 60
DT = 1.0 / FPS
ORIGIN = (WIDTH // 2, HEIGHT - 120)    # pixel position of the arm base

L1 = 1.5                               # link 1 length [m]
L2 = 1.0                               # link 2 length [m]
JOINT_SPEED = 1.0                      # manual joint speed [rad/s]

Q_A = (math.radians(20.0), math.radians(60.0))
Q_B = (math.radians(110.0), math.radians(-50.0))
MOVE_TIME = 3.0                        # duration T of the A -> B motion [s]


# ----------------------------------------------------------------------------
# Coordinate helpers
# ----------------------------------------------------------------------------
def world_to_screen(x, y):
    """Metres (y up, origin at base) -> pixels (y down)."""
    return int(ORIGIN[0] + x * SCALE), int(ORIGIN[1] - y * SCALE)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ----------------------------------------------------------------------------
# Kinematics
# ----------------------------------------------------------------------------
def fk(q1, q2):
    """Forward kinematics[cite: 7]."""
    ex = L1 * math.cos(q1)
    ey = L1 * math.sin(q1)
    tx = ex + L2 * math.cos(q1 + q2)
    ty = ey + L2 * math.sin(q1 + q2)
    return (ex, ey), (tx, ty)


def ik(x, y, elbow_up=True):
    """Analytic inverse kinematics for planar 2R arm[cite: 3, 7]."""
    r_sq = x**2 + y**2
    cos_q2 = (r_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    
    # Check reachability bounds
    if abs(cos_q2) > 1.0:
        return None
        
    # Choose elbow configuration sign
    q2 = math.acos(cos_q2)
    if not elbow_up:
        q2 = -q2
        
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


def jacobian_det(q2):
    """det J of the planar 2R arm[cite: 3, 7]."""
    return L1 * L2 * math.sin(q2)


# ----------------------------------------------------------------------------
# Motion generation
# ----------------------------------------------------------------------------
def time_scaling(t, T, kind="cubic"):
    """Map time t in [0, T] to progress s in [0, 1] with profile choices[cite: 3, 7]."""
    u = clamp(t / T, 0.0, 1.0)
    if kind == "linear":
        return u
    elif kind == "cubic":
        return 3.0 * (u**2) - 2.0 * (u**3)
    elif kind == "trapezoidal":
        # Simple trapezoidal profile approximation (acceleration/deceleration blending)
        tb = 0.25  # blend fraction
        if u < tb:
            return 0.5 * (u / tb)**2 * tb
        elif u > (1.0 - tb):
            u_rem = 1.0 - u
            return 1.0 - 0.5 * (u_rem / tb)**2 * tb
        else:
            return u - 0.5 * tb
    return u


def joint_space_q(t):
    """Joint angles via joint-space interpolation[cite: 3, 7]."""
    s = time_scaling(t, MOVE_TIME, kind="cubic")
    q1 = Q_A[0] + s * (Q_B[0] - Q_A[0])
    q2 = Q_A[1] + s * (Q_B[1] - Q_A[1])
    return q1, q2


def cartesian_line_q(t):
    """Joint angles for a straight-line Cartesian move A -> B using IK[cite: 3, 7]."""
    _, A_xy = fk(*Q_A)
    _, B_xy = fk(*Q_B)
    
    s = time_scaling(t, MOVE_TIME, kind="cubic")
    x = A_xy[0] + s * (B_xy[0] - A_xy[0])
    y = A_xy[1] + s * (B_xy[1] - A_xy[1])
    
    sol = ik(x, y, elbow_up=True)
    if sol is None:
        return Q_A  # Fallback if outside workspace boundary
    return sol


# ----------------------------------------------------------------------------
# Drawing
# ----------------------------------------------------------------------------
BG = (245, 245, 245)
GRID = (225, 225, 225)
LINK1 = (40, 90, 170)
LINK2 = (60, 140, 200)
JOINT = (30, 30, 30)
TIP = (230, 80, 40)
MARK_A = (30, 160, 70)
MARK_B = (170, 40, 160)
TRAIL_COLOR = (70, 130, 180)


def draw_grid(screen):
    for k in range(-4, 5):
        x0, _ = world_to_screen(k, 0)
        pygame.draw.line(screen, GRID, (x0, 0), (x0, HEIGHT))
    for k in range(-1, 5):
        _, y0 = world_to_screen(0, k)
        pygame.draw.line(screen, GRID, (0, y0), (WIDTH, y0))


def draw_arm(screen, q1, q2):
    elbow, tip = fk(q1, q2)
    base_px = world_to_screen(0, 0)
    elbow_px = world_to_screen(*elbow)
    tip_px = world_to_screen(*tip)
    pygame.draw.rect(screen, JOINT, (base_px[0] - 30, base_px[1] + 5, 60, 14))
    pygame.draw.line(screen, LINK1, base_px, elbow_px, 10)
    pygame.draw.line(screen, LINK2, elbow_px, tip_px, 8)
    pygame.draw.circle(screen, JOINT, base_px, 9)
    pygame.draw.circle(screen, JOINT, elbow_px, 8)
    pygame.draw.circle(screen, TIP, tip_px, 7)


def draw_trail(screen, trail):
    """Draw end-effector trail[cite: 4, 7]."""
    if len(trail) >= 2:
        pixel_points = [world_to_screen(x, y) for x, y in trail]
        pygame.draw.lines(screen, TRAIL_COLOR, False, pixel_points, 2)


def draw_markers(screen):
    """Mark start (A) and end (B) end-effector positions[cite: 3, 7]."""
    _, pos_a = fk(*Q_A)
    _, pos_b = fk(*Q_B)
    
    px_a = world_to_screen(*pos_a)
    px_b = world_to_screen(*pos_b)
    
    pygame.draw.circle(screen, MARK_A, px_a, 6, 2)
    pygame.draw.circle(screen, MARK_B, px_b, 6, 2)


def draw_workspace(screen):
    """Sample joint space to render reachable workspace points and boundary limits[cite: 3, 7]."""
    step = 0.15
    q1_vals = [i * step for i in range(int(-math.pi / step), int(math.pi / step) + 1)]
    q2_vals = [i * step for i in range(int(-math.pi / step), int(math.pi / step) + 1)]
    
    for q1 in q1_vals:
        for q2 in q2_vals:
            _, tip = fk(q1, q2)
            pt = world_to_screen(*tip)
            if 0 <= pt[0] <= WIDTH and 0 <= pt[1] <= HEIGHT:
                screen.set_at(pt, (210, 210, 230))


def draw_hud(screen, font, q1, q2, mode, moving):
    _, tip = fk(q1, q2)
    lines = [f"mode: {mode}" + ("   (moving)" if moving else ""),
             f"q1 = {math.degrees(q1):7.2f} deg    q2 = {math.degrees(q2):7.2f} deg",
             f"End-Effector: x = {tip[0]:5.3f} m, y = {tip[1]:5.3f} m"]
    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, (20, 20, 20)), (10, 8 + 20 * i))


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def main():
    if pygame is None:
        raise SystemExit("pygame is required to run this program. Install it with: pip install pygame")

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("CS2110 - Planar 2R Arm Kinematics")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas,menlo,monospace", 16)

    q1, q2 = Q_A
    mode = "MANUAL"
    moving = False
    t = 0.0
    trail = []
    show_workspace = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_1:
                    mode, moving = "MANUAL", False
                elif event.key == pygame.K_2:
                    mode, moving = "JOINT_A_TO_B", False
                    q1, q2 = Q_A
                elif event.key == pygame.K_3:
                    mode, moving = "CARTESIAN_A_TO_B", False
                    q1, q2 = Q_A
                elif event.key == pygame.K_SPACE and mode != "MANUAL":
                    q1, q2 = Q_A
                    t, moving = 0.0, True
                    trail.clear()
                elif event.key == pygame.K_w:
                    show_workspace = not show_workspace
                elif event.key == pygame.K_c:
                    trail.clear()

        # ---- update joint angles ----
        if mode == "MANUAL":
            keys = pygame.key.get_pressed()
            q1 += (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT]) * JOINT_SPEED * DT
            q2 += (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * JOINT_SPEED * DT
        elif moving:
            t += DT
            if mode == "JOINT_A_TO_B":
                q1, q2 = joint_space_q(t)
            else:
                q1, q2 = cartesian_line_q(t)
            
            _, tip = fk(q1, q2)
            if len(trail) == 0 or math.hypot(tip[0] - trail[-1][0], tip[1] - trail[-1][1]) > 0.02:
                trail.append(tip)
                
            if t >= MOVE_TIME:
                moving = False

        # ---- draw ----
        screen.fill(BG)
        draw_grid(screen)
        if show_workspace:
            draw_workspace(screen)
        draw_trail(screen, trail)
        draw_markers(screen)
        draw_arm(screen, q1, q2)
        draw_hud(screen, font, q1, q2, mode, moving)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()