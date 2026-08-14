import pygame
import math

pygame.init()

# ============================================================
# WINDOW
# ============================================================

WIDTH = 900
HEIGHT = 650

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("RoboPilot - Autonomous Robot Simulator")

clock = pygame.time.Clock()

# ============================================================
# COLORS
# ============================================================

BACKGROUND = (30, 30, 30)
WALL_COLOR = (180, 180, 180)
ROBOT_COLOR = (50, 150, 255)
TARGET_COLOR = (50, 220, 80)
SENSOR_COLOR = (255, 220, 50)
TEXT_COLOR = (240, 240, 240)
DANGER_COLOR = (255, 80, 80)

# ============================================================
# ROBOT
# ============================================================

robot_x = 120
robot_y = 500

robot_radius = 25
speed = 3

# Robot direction
# 0 degrees = right
robot_angle = 0

# ============================================================
# TARGET
# ============================================================

target_x = 760
target_y = 120

target_radius = 20
target_active = True

# Does the robot currently see the target?
target_visible = False

# ============================================================
# WALLS
# ============================================================

walls = [
    pygame.Rect(300, 120, 35, 350),
    pygame.Rect(520, 250, 35, 300),
    pygame.Rect(650, 80, 180, 35),
    pygame.Rect(650, 500, 180, 35)
]

# ============================================================
# SENSORS
# ============================================================

# Five sensors:
#
#             -60   -30   0   +30   +60
#               \    \    |    /    /
#                \    \   |   /    /
#                     ROBOT

sensor_angles = [-60, -30, 0, 30, 60]

sensor_range = 180

danger_distance = 55

sensor_distances = [sensor_range] * len(sensor_angles)

# ============================================================
# MODE
# ============================================================

autonomous = False

# Used while searching for the target
search_turn_timer = 0


# ============================================================
# COLLISION DETECTION
# ============================================================

def is_collision(x, y):

    # Screen boundaries

    if x - robot_radius < 0:
        return True

    if x + robot_radius > WIDTH:
        return True

    if y - robot_radius < 0:
        return True

    if y + robot_radius > HEIGHT:
        return True

    # Walls

    for wall in walls:

        expanded_wall = wall.inflate(
            robot_radius * 2,
            robot_radius * 2
        )

        if expanded_wall.collidepoint(x, y):
            return True

    return False


# ============================================================
# DISTANCE
# ============================================================

def distance_between(x1, y1, x2, y2):

    dx = x2 - x1
    dy = y2 - y1

    return math.sqrt(dx ** 2 + dy ** 2)


# ============================================================
# SENSOR
# ============================================================

def read_sensor(sensor_angle):

    angle = math.radians(
        robot_angle + sensor_angle
    )

    # Move outward from the robot
    # a few pixels at a time.

    for distance in range(0, sensor_range, 4):

        sensor_x = (
            robot_x
            + math.cos(angle) * distance
        )

        sensor_y = (
            robot_y
            + math.sin(angle) * distance
        )

        # Screen edge

        if (
            sensor_x < 0
            or sensor_x >= WIDTH
            or sensor_y < 0
            or sensor_y >= HEIGHT
        ):
            return distance

        # Walls

        for wall in walls:

            if wall.collidepoint(
                sensor_x,
                sensor_y
            ):
                return distance

    return sensor_range


# ============================================================
# UPDATE ALL SENSORS
# ============================================================

def update_sensors():

    for i in range(len(sensor_angles)):

        sensor_distances[i] = read_sensor(
            sensor_angles[i]
        )


# ============================================================
# CAN ROBOT SEE TARGET?
# ============================================================

def can_see_target():

    if not target_active:
        return False

    # Distance to target

    dx = target_x - robot_x
    dy = target_y - robot_y

    target_distance = math.sqrt(
        dx ** 2 + dy ** 2
    )

    # Target is outside sensor range

    if target_distance > sensor_range:
        return False

    # Direction from robot to target

    target_angle = math.degrees(
        math.atan2(dy, dx)
    )

    # Difference between robot direction
    # and target direction

    angle_difference = (
        target_angle - robot_angle
    )

    # Keep angle between -180 and +180

    while angle_difference > 180:
        angle_difference -= 360

    while angle_difference < -180:
        angle_difference += 360

    # Target must be within 60 degrees
    # of the robot's direction.

    if abs(angle_difference) > 60:
        return False

    # --------------------------------------------------------
    # CHECK FOR WALL BETWEEN ROBOT AND TARGET
    # --------------------------------------------------------

    for distance in range(
        0,
        int(target_distance),
        4
    ):

        check_x = (
            robot_x
            + math.cos(
                math.radians(target_angle)
            ) * distance
        )

        check_y = (
            robot_y
            + math.sin(
                math.radians(target_angle)
            ) * distance
        )

        for wall in walls:

            if wall.collidepoint(
                check_x,
                check_y
            ):
                return False

    return True


# ============================================================
# AUTONOMOUS CONTROL
# ============================================================

def autonomous_control():

    global robot_x
    global robot_y
    global robot_angle
    global search_turn_timer

    front_left = sensor_distances[1]
    front = sensor_distances[2]
    front_right = sensor_distances[3]

    # ========================================================
    # PRIORITY 1 — AVOID OBSTACLES
    # ========================================================

    if front < danger_distance:

        # Turn toward the side with more space.

        if front_left > front_right:

            robot_angle -= 6

        else:

            robot_angle += 6

        return

    # ========================================================
    # PRIORITY 2 — TARGET DETECTED
    # ========================================================

    if can_see_target():

        dx = target_x - robot_x
        dy = target_y - robot_y

        target_angle = math.degrees(
            math.atan2(dy, dx)
        )

        angle_difference = (
            target_angle - robot_angle
        )

        # Normalize angle

        while angle_difference > 180:
            angle_difference -= 360

        while angle_difference < -180:
            angle_difference += 360

        # Turn toward target

        if angle_difference > 4:

            robot_angle += 3

        elif angle_difference < -4:

            robot_angle -= 3

        # Move forward

        radians = math.radians(robot_angle)

        new_x = (
            robot_x
            + math.cos(radians) * speed
        )

        new_y = (
            robot_y
            + math.sin(radians) * speed
        )

        if not is_collision(new_x, new_y):

            robot_x = new_x
            robot_y = new_y

        return

    # ========================================================
    # PRIORITY 3 — SEARCH
    # ========================================================

    search_turn_timer += 1

    # Occasionally change direction

    if search_turn_timer > 100:

        robot_angle += 45

        search_turn_timer = 0

    # Move forward

    radians = math.radians(robot_angle)

    new_x = (
        robot_x
        + math.cos(radians) * speed
    )

    new_y = (
        robot_y
        + math.sin(radians) * speed
    )

    if not is_collision(new_x, new_y):

        robot_x = new_x
        robot_y = new_y

    else:

        # Something blocked the robot.
        # Turn away.

        robot_angle += 90


# ============================================================
# MANUAL CONTROL
# ============================================================

def manual_control(keys):

    global robot_x
    global robot_y
    global robot_angle

    # Rotate

    if keys[pygame.K_LEFT]:

        robot_angle -= 4

    if keys[pygame.K_RIGHT]:

        robot_angle += 4

    radians = math.radians(robot_angle)

    # Forward

    if keys[pygame.K_UP]:

        new_x = (
            robot_x
            + math.cos(radians) * speed
        )

        new_y = (
            robot_y
            + math.sin(radians) * speed
        )

        if not is_collision(new_x, new_y):

            robot_x = new_x
            robot_y = new_y

    # Backward

    if keys[pygame.K_DOWN]:

        new_x = (
            robot_x
            - math.cos(radians) * speed
        )

        new_y = (
            robot_y
            - math.sin(radians) * speed
        )

        if not is_collision(new_x, new_y):

            robot_x = new_x
            robot_y = new_y


# ============================================================
# RESET
# ============================================================

def reset_robot():

    global robot_x
    global robot_y
    global robot_angle
    global target_active
    global search_turn_timer

    robot_x = 120
    robot_y = 500

    robot_angle = 0

    target_active = True

    search_turn_timer = 0


# ============================================================
# DRAW SENSORS
# ============================================================

def draw_sensors():

    for i in range(len(sensor_angles)):

        sensor_angle = sensor_angles[i]

        distance = sensor_distances[i]

        angle = math.radians(
            robot_angle + sensor_angle
        )

        end_x = (
            robot_x
            + math.cos(angle) * distance
        )

        end_y = (
            robot_y
            + math.sin(angle) * distance
        )

        # Red = danger
        # Yellow = safe

        if distance < danger_distance:

            color = DANGER_COLOR

        else:

            color = SENSOR_COLOR

        pygame.draw.line(
            screen,
            color,
            (robot_x, robot_y),
            (end_x, end_y),
            2
        )

        pygame.draw.circle(
            screen,
            color,
            (
                int(end_x),
                int(end_y)
            ),
            4
        )


# ============================================================
# DRAW ROBOT
# ============================================================

def draw_robot():

    # Robot body

    pygame.draw.circle(
        screen,
        ROBOT_COLOR,
        (
            int(robot_x),
            int(robot_y)
        ),
        robot_radius
    )

    # Direction indicator

    angle = math.radians(robot_angle)

    direction_x = (
        robot_x
        + math.cos(angle) * robot_radius
    )

    direction_y = (
        robot_y
        + math.sin(angle) * robot_radius
    )

    pygame.draw.line(
        screen,
        (255, 255, 255),
        (
            int(robot_x),
            int(robot_y)
        ),
        (
            int(direction_x),
            int(direction_y)
        ),
        4
    )


# ============================================================
# TEXT
# ============================================================

font = pygame.font.SysFont(
    None,
    24
)


def draw_text():

    if autonomous:

        mode = "AUTONOMOUS"

    else:

        mode = "MANUAL"

    screen.blit(
        font.render(
            "Mode: " + mode,
            True,
            TEXT_COLOR
        ),
        (15, 15)
    )

    screen.blit(
        font.render(
            "A: Autonomous   M: Manual   R: Reset   ESC: Quit",
            True,
            TEXT_COLOR
        ),
        (15, 40)
    )

    # Sensor values

    sensor_text = (
        "Sensors: "
        + str(int(sensor_distances[0]))
        + "  "
        + str(int(sensor_distances[1]))
        + "  "
        + str(int(sensor_distances[2]))
        + "  "
        + str(int(sensor_distances[3]))
        + "  "
        + str(int(sensor_distances[4]))
    )

    screen.blit(
        font.render(
            sensor_text,
            True,
            TEXT_COLOR
        ),
        (15, 65)
    )

    # Target status

    if target_visible:

        status = "TARGET DETECTED!"

    else:

        status = "SEARCHING..."

    screen.blit(
        font.render(
            status,
            True,
            TEXT_COLOR
        ),
        (15, 90)
    )


# ============================================================
# MAIN LOOP
# ============================================================

running = True

while running:

    # ========================================================
    # EVENTS
    # ========================================================

    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            running = False

        if event.type == pygame.KEYDOWN:

            if event.key == pygame.K_ESCAPE:

                running = False

            if event.key == pygame.K_a:

                autonomous = True

            if event.key == pygame.K_m:

                autonomous = False

            if event.key == pygame.K_r:

                reset_robot()

    # ========================================================
    # KEYBOARD
    # ========================================================

    keys = pygame.key.get_pressed()

    # ========================================================
    # SENSORS
    # ========================================================

    update_sensors()

    # ========================================================
    # TARGET VISIBILITY
    # ========================================================

    target_visible = can_see_target()

    # ========================================================
    # ROBOT CONTROL
    # ========================================================

    if autonomous:

        if target_active:

            autonomous_control()

    else:

        manual_control(keys)

    # ========================================================
    # TARGET DISTANCE
    # ========================================================

    distance_to_target = distance_between(
        robot_x,
        robot_y,
        target_x,
        target_y
    )

    # ========================================================
    # TARGET REACHED
    # ========================================================

    if target_active:

        if distance_to_target < (
            robot_radius + target_radius
        ):

            target_active = False

            print("Target reached!")

    # ========================================================
    # DRAW
    # ========================================================

    screen.fill(BACKGROUND)

    # Walls

    for wall in walls:

        pygame.draw.rect(
            screen,
            WALL_COLOR,
            wall
        )

    # Target

    if target_active:

        pygame.draw.circle(
            screen,
            TARGET_COLOR,
            (
                target_x,
                target_y
            ),
            target_radius
        )

    # Sensors

    draw_sensors()

    # Robot

    draw_robot()

    # Information

    draw_text()

    # ========================================================
    # UPDATE DISPLAY
    # ========================================================

    pygame.display.flip()

    clock.tick(60)


pygame.quit()