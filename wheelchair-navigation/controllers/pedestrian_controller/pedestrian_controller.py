from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
node = robot.getSelf()
translation = node.getField('translation')
rotation = node.getField('rotation')
name = robot.getName()

if 'crossing' in name:
    # Moves back and forth across the crosswalk.
    y = 0.0
    z = 0.8
    amplitude = 5.8
    period = 9.0
    while robot.step(timestep) != -1:
        t = robot.getTime()
        x = amplitude * math.cos(2 * math.pi * t / period)
        translation.setSFVec3f([x, y, z])
        direction = -1 if math.sin(2 * math.pi * t / period) > 0 else 1
        rotation.setSFRotation([0, 0, 1, 1.5708 if direction > 0 else -1.5708])
else:
    # Walks slowly along the sidewalk.
    x = 5.7
    z = 0.8
    center_y = 4.0
    amplitude = 3.2
    period = 12.0
    while robot.step(timestep) != -1:
        t = robot.getTime()
        y = center_y + amplitude * math.sin(2 * math.pi * t / period)
        translation.setSFVec3f([x, y, z])
        direction = 1 if math.cos(2 * math.pi * t / period) > 0 else -1
        rotation.setSFRotation([0, 0, 1, 0 if direction > 0 else 3.14159])
