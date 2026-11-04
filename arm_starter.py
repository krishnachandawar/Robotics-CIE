
import math
import sys
import pygame

# ----------------------------------------------------------------------------
# Parameters  (Part 2.3: you will vary these)
# ----------------------------------------------------------------------------
WIDTH, HEIGHT = 900, 650
SCALE = 100.0                          # pixels per metre  (100 px = 1 m)
FPS = 60
DT = 1.0 / FPS
ORIGIN = (WIDTH // 2, HEIGHT - 140)    # pixel position of the arm base

L1 = 1.5                               # link 1 length [m]
L2 = 1.0                               # link 2 length [m]
JOINT_SPEED = 1.2                      # manual joint speed [rad/s]

# Start (A) and end (B) configurations, given as joint angles [rad]
Q_A = (math.radians(20.0), math.radians(60.0))
Q_B = (math.radians(110.0), math.radians(-50.0))
MOVE_TIME = 3.0                        # duration T of the A -> B motion [s]


# ----------------------------------------------------------------------------
# Coordinate helpers (working)
# ----------------------------------------------------------------------------
def world_to_screen(x, y):
    """Metres (y up, origin at base) -> pixels (y down)."""
    return int(ORIGIN[0] + x * SCALE), int(ORIGIN[1] - y * SCALE)


def screen_to_world(px, py):
    """Pixels -> metres (origin at base, y up)."""
    return (px - ORIGIN[0]) / SCALE, (ORIGIN[1] - py) / SCALE


def clamp(value, lo, hi):
    """Clamp value to range [lo, hi]."""
    return max(lo, min(hi, value))


def wrap_angle(a):
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ----------------------------------------------------------------------------
# Kinematics
# ----------------------------------------------------------------------------
def fk(q1, q2, l1=L1, l2=L2):
    """Forward Kinematics (FK):
         Elbow: (ex, ey) = (L1 * cos(q1), L1 * sin(q1))
         Tip:   (tx, ty) = (ex + L2 * cos(q1 + q2), ey + L2 * sin(q1 + q2))
    Returns (elbow_xy, end_effector_xy).
    """
    ex = l1 * math.cos(q1)
    ey = l1 * math.sin(q1)
    tx = ex + l2 * math.cos(q1 + q2)
    ty = ey + l2 * math.sin(q1 + q2)
    return (ex, ey), (tx, ty)


def ik(x, y, elbow_up=False, l1=L1, l2=L2):
    """Analytical Inverse Kinematics (IK) (Part 2.5):
         cos(q2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
         q2      = +/- acos(cos_q2)
         q1      = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2))

    Returns (q1, q2) or None if target is outside reachable workspace.
    Convention:
      elbow_up=False (elbow down) -> q2 > 0 (matches start pose Q_A = (20 deg, +60 deg))
      elbow_up=True  (elbow up)   -> q2 < 0 (matches pose Q_B = (110 deg, -50 deg))
    """
    d_sq = x * x + y * y
    c2 = (d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)

    # Check reachability boundary condition
    if abs(c2) > 1.0:
        return None  # Target point is strictly unreachable

    s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
    q2 = -math.atan2(s2, c2) if elbow_up else math.atan2(s2, c2)

    k1 = l1 + l2 * math.cos(q2)
    k2 = l2 * math.sin(q2)
    q1 = math.atan2(y, x) - math.atan2(k2, k1)

    return q1, q2


def jacobian_det(q2, l1=L1, l2=L2):
    """Jacobian determinant: det(J) = L1 * L2 * sin(q2).
    Singularities occur at q2 = 0 (fully outstretched, boundary r = L1+L2)
    and q2 = pi (folded back, boundary r = |L1-L2|).
    """
    return l1 * l2 * math.sin(q2)


# ----------------------------------------------------------------------------
# Motion generation & Time scaling (Part 2.3)
# ----------------------------------------------------------------------------
def time_scaling(t, T, kind="cubic"):
    """Map time t in [0, T] to progress s in [0, 1].

    - 'linear': s = u  (discontinuous acceleration at boundaries)
    - 'cubic': s = 3*u^2 - 2*u^3  (smooth zero velocity at t=0 and t=T)
    - 'trapezoidal': piecewise quadratic/linear velocity profile with 1/3 blend times
    """
    u = clamp(t / T, 0.0, 1.0)
    if kind == "linear":
        return u
    elif kind == "cubic":
        return 3.0 * (u ** 2) - 2.0 * (u ** 3)
    elif kind == "trapezoidal":
        # 1/3 acceleration, 1/3 constant cruise, 1/3 deceleration
        if u < (1.0 / 3.0):
            return 0.5 * 4.5 * (u ** 2)
        elif u < (2.0 / 3.0):
            return 0.25 + 1.5 * (u - 1.0 / 3.0)
        else:
            return 1.0 - 0.5 * 4.5 * ((1.0 - u) ** 2)
    return u


def joint_space_q(t, kind="cubic"):
    """Joint angles at time t for the A -> B move (Part 2.2).
    Linear joint-space interpolation: q(t) = q_A + s(t) * (q_B - q_A)
    """
    s = time_scaling(t, MOVE_TIME, kind=kind)
    q1 = Q_A[0] + s * (Q_B[0] - Q_A[0])
    q2 = Q_A[1] + s * (Q_B[1] - Q_A[1])
    return q1, q2


def cartesian_line_q(t, elbow_up=False, kind="cubic"):
    """Joint angles for a straight-line Cartesian move A -> B (Part 2.5).

    1. Compute start and goal Cartesian positions: A_xy = fk(*Q_A)[1], B_xy = fk(*Q_B)[1]
    2. Interpolate Cartesian position: P(t) = A_xy + s(t) * (B_xy - A_xy)
    3. Solve Inverse Kinematics: (q1, q2) = ik(P(t))
    """
    A_xy = fk(*Q_A)[1]
    B_xy = fk(*Q_B)[1]
    s = time_scaling(t, MOVE_TIME, kind=kind)
    xt = A_xy[0] + s * (B_xy[0] - A_xy[0])
    yt = A_xy[1] + s * (B_xy[1] - A_xy[1])

    sol = ik(xt, yt, elbow_up=elbow_up)
    if sol is not None:
        return sol
    # Fallback to joint space if unreachable (boundary edge safety)
    return joint_space_q(t, kind=kind)


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
TRAIL_COLOR = (220, 80, 50)
CHORD_COLOR = (150, 150, 150)
WORKSPACE_COLOR = (210, 230, 250)


def draw_grid(screen):
    for k in range(-5, 6):
        x0, _ = world_to_screen(k, 0)
        pygame.draw.line(screen, GRID, (x0, 0), (x0, HEIGHT))
    for k in range(-2, 5):
        _, y0 = world_to_screen(0, k)
        pygame.draw.line(screen, GRID, (0, y0), (WIDTH, y0))
    # Origin marker
    pygame.draw.circle(screen, (180, 180, 180), ORIGIN, 4)


def draw_arm(screen, q1, q2):
    elbow, tip = fk(q1, q2)
    base_px = world_to_screen(0, 0)
    elbow_px = world_to_screen(*elbow)
    tip_px = world_to_screen(*tip)

    # Base mounting pedestal
    pygame.draw.rect(screen, JOINT, (base_px[0] - 35, base_px[1] + 5, 70, 14), border_radius=3)
    # Link 1 and Link 2
    pygame.draw.line(screen, LINK1, base_px, elbow_px, 10)
    pygame.draw.line(screen, LINK2, elbow_px, tip_px, 8)
    # Joint hubs
    pygame.draw.circle(screen, JOINT, base_px, 10)
    pygame.draw.circle(screen, (200, 200, 200), base_px, 4)
    pygame.draw.circle(screen, JOINT, elbow_px, 9)
    pygame.draw.circle(screen, (200, 200, 200), elbow_px, 3)
    # End-effector tool tip
    pygame.draw.circle(screen, TIP, tip_px, 8)
    pygame.draw.circle(screen, (255, 255, 255), tip_px, 3)


def draw_trail(screen, trail):
    """Part 2.2: Draw the end-effector trajectory trail."""
    if len(trail) >= 2:
        pixel_points = [world_to_screen(x, y) for x, y in trail]
        pygame.draw.lines(screen, TRAIL_COLOR, False, pixel_points, 3)


def draw_markers(screen):
    """Part 2.2: Mark end-effector positions A and B, plus the straight chord line."""
    font = pygame.font.SysFont("consolas,menlo,monospace", 14, bold=True)
    A_xy = fk(*Q_A)[1]
    B_xy = fk(*Q_B)[1]
    A_px = world_to_screen(*A_xy)
    B_px = world_to_screen(*B_xy)

    # Draw straight chord reference line connecting A and B
    pygame.draw.line(screen, CHORD_COLOR, A_px, B_px, 1)

    # Marker A
    pygame.draw.circle(screen, MARK_A, A_px, 7)
    pygame.draw.circle(screen, (255, 255, 255), A_px, 2)
    label_a = font.render(f"A ({A_xy[0]:.2f}, {A_xy[1]:.2f})", True, MARK_A)
    screen.blit(label_a, (A_px[0] + 10, A_px[1] - 8))

    # Marker B
    pygame.draw.circle(screen, MARK_B, B_px, 7)
    pygame.draw.circle(screen, (255, 255, 255), B_px, 2)
    label_b = font.render(f"B ({B_xy[0]:.2f}, {B_xy[1]:.2f})", True, MARK_B)
    screen.blit(label_b, (B_px[0] + 10, B_px[1] - 8))


# Pre-generate workspace surface for instant, stutter-free display
_workspace_surface = None

def get_workspace_surface():
    global _workspace_surface
    if _workspace_surface is not None:
        return _workspace_surface

    surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    # Sample (q1, q2) on a grid
    step_q1 = math.radians(6.0)
    step_q2 = math.radians(6.0)
    for i in range(int(360 / 6)):
        q1 = i * step_q1
        for j in range(int(360 / 6)):
            q2 = j * step_q2
            _, tip = fk(q1, q2)
            px, py = world_to_screen(*tip)
            if 0 <= px < WIDTH and 0 <= py < HEIGHT:
                surf.set_at((px, py), (170, 205, 240, 110))

    # Draw boundary circles
    base_px = world_to_screen(0, 0)
    r_outer_px = int((L1 + L2) * SCALE)
    r_inner_px = int(abs(L1 - L2) * SCALE)

    pygame.draw.circle(surf, (70, 130, 210, 180), base_px, r_outer_px, 2)
    pygame.draw.circle(surf, (220, 80, 80, 180), base_px, r_inner_px, 2)
    _workspace_surface = surf
    return _workspace_surface


def draw_workspace(screen):
    """Part 2.4: Render reachable workspace cloud and boundaries."""
    screen.blit(get_workspace_surface(), (0, 0))
    # Legend for workspace boundaries
    font = pygame.font.SysFont("consolas,menlo,monospace", 13)
    screen.blit(font.render(f"Outer Boundary (q2=0, det J=0): r_max = {L1 + L2:.1f} m", True, (40, 100, 190)), (WIDTH - 420, 15))
    screen.blit(font.render(f"Inner Boundary (q2=pi, det J=0): r_min = {abs(L1 - L2):.1f} m", True, (210, 60, 60)), (WIDTH - 420, 33))


def compute_metrics(trail):
    """Compute path length and maximum deviation from straight chord AB."""
    if len(trail) < 2:
        return 0.0, 0.0
    A_xy = fk(*Q_A)[1]
    B_xy = fk(*Q_B)[1]
    chord_len = math.hypot(B_xy[0] - A_xy[0], B_xy[1] - A_xy[1])
    dx_AB = B_xy[0] - A_xy[0]
    dy_AB = B_xy[1] - A_xy[1]

    path_len = 0.0
    max_dev = 0.0
    for i in range(1, len(trail)):
        p1 = trail[i - 1]
        p2 = trail[i]
        path_len += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if chord_len > 1e-6:
            dev = abs(dx_AB * (A_xy[1] - p2[1]) - (A_xy[0] - p2[0]) * dy_AB) / chord_len
            if dev > max_dev:
                max_dev = dev
    return path_len, max_dev


def draw_hud(screen, font, q1, q2, mode, moving, profile, elbow_up, trail):
    """Part 2.1: Comprehensive HUD displaying joints, Cartesian tip, Jacobian, metrics."""
    elbow, tip = fk(q1, q2)
    detJ = jacobian_det(q2)
    path_len, max_dev = compute_metrics(trail)

    lines = [
        f"Mode: {mode}" + ("  [MOVING]" if moving else "  [IDLE]"),
        f"Joints: q1 = {math.degrees(q1):6.1f} deg ({q1:5.2f} rad),  q2 = {math.degrees(q2):6.1f} deg ({q2:5.2f} rad)",
        f"Tip (x, y): ({tip[0]:5.3f}, {tip[1]:5.3f}) m  |  Radius: {math.hypot(*tip):5.3f} m",
        f"det(J) = {detJ:6.3f}  {'[SINGULARITY!]' if abs(detJ) < 0.05 else ''}",
        f"Profile: {profile.upper()}  |  IK Config: {'ELBOW UP' if elbow_up else 'ELBOW DOWN'}",
        f"Path Length: {path_len:5.3f} m  |  Max Deviation: {max_dev:5.3f} m",
    ]

    hud_surf = pygame.Surface((560, 24 + 20 * len(lines)))
    hud_surf.set_alpha(200)
    hud_surf.fill((255, 255, 255))
    screen.blit(hud_surf, (8, 6))

    for i, text in enumerate(lines):
        color = (190, 40, 20) if "SINGULARITY" in text else (20, 20, 20)
        screen.blit(font.render(text, True, color), (14, 10 + 20 * i))

    help_lines = [
        "Keys: 1=Manual | 2=Joint A->B | 3=Cartesian Straight Line | 4=Mouse IK",
        "SPACE=Start Motion | P=Cycle Profile | E=Toggle Elbow | W=Workspace | C=Clear Trail"
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
    pygame.display.set_caption("CS2110 - Planar 2R Arm Kinematics Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas,menlo,monospace", 15)

    q1, q2 = Q_A
    mode = "MANUAL"
    moving = False
    t = 0.0
    trail = [fk(q1, q2)[1]]
    show_workspace = False
    profile = "cubic"           # "linear", "cubic", or "trapezoidal"
    elbow_up = False            # False: elbow down (matches Q_A)

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
                    trail = [fk(q1, q2)[1]]
                elif event.key == pygame.K_3:
                    mode, moving = "CARTESIAN_A_TO_B", False
                    q1, q2 = Q_A
                    trail = [fk(q1, q2)[1]]
                elif event.key == pygame.K_4:
                    mode, moving = "MOUSE_IK", False
                elif event.key == pygame.K_SPACE and mode in ("JOINT_A_TO_B", "CARTESIAN_A_TO_B"):
                    q1, q2 = Q_A
                    t, moving = 0.0, True
                    trail = [fk(q1, q2)[1]]
                elif event.key == pygame.K_p:
                    # Cycle time-scaling profile
                    profs = ["linear", "cubic", "trapezoidal"]
                    profile = profs[(profs.index(profile) + 1) % len(profs)]
                elif event.key == pygame.K_e:
                    elbow_up = not elbow_up
                elif event.key == pygame.K_w:
                    show_workspace = not show_workspace
                elif event.key == pygame.K_c:
                    trail.clear()
                    trail.append(fk(q1, q2)[1])

        # ---- update joint angles ----
        if mode == "MANUAL":
            keys = pygame.key.get_pressed()
            q1 += (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT]) * JOINT_SPEED * DT
            q2 += (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * JOINT_SPEED * DT
            trail.append(fk(q1, q2)[1])
        elif mode == "MOUSE_IK":
            m_px, m_py = pygame.mouse.get_pos()
            mx, my = screen_to_world(m_px, m_py)
            sol = ik(mx, my, elbow_up=elbow_up)
            if sol is not None:
                q1, q2 = sol
                trail.append(fk(q1, q2)[1])
        elif moving:
            t += DT
            if mode == "JOINT_A_TO_B":
                q1, q2 = joint_space_q(t, kind=profile)
            elif mode == "CARTESIAN_A_TO_B":
                q1, q2 = cartesian_line_q(t, elbow_up=elbow_up, kind=profile)

            trail.append(fk(q1, q2)[1])
            if t >= MOVE_TIME:
                moving = False

        # Limit trail size for performance
        if len(trail) > 1500:
            trail = trail[-1500:]

        # ---- draw ----
        screen.fill(BG)
        draw_grid(screen)
        if show_workspace:
            draw_workspace(screen)
        draw_trail(screen, trail)
        draw_markers(screen)
        draw_arm(screen, q1, q2)
        draw_hud(screen, font, q1, q2, mode, moving, profile, elbow_up, trail)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
