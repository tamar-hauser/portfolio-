from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

red = robot.getFromDef('RED_LIGHT_APPEARANCE')
yellow = robot.getFromDef('YELLOW_LIGHT_APPEARANCE')
green = robot.getFromDef('GREEN_LIGHT_APPEARANCE')

states = [
    ('green', 6.0),
    ('yellow', 2.0),
    ('red', 6.0),
]

def set_color(active):
    def set_app(app, base, emissive):
        if app:
            app.getField('baseColor').setSFColor(base)
            app.getField('emissiveColor').setSFColor(emissive)
    set_app(red, [1, 0, 0] if active == 'red' else [0.20, 0, 0], [1, 0, 0] if active == 'red' else [0, 0, 0])
    set_app(yellow, [1, 0.75, 0] if active == 'yellow' else [0.20, 0.15, 0], [1, 0.5, 0] if active == 'yellow' else [0, 0, 0])
    set_app(green, [0, 1, 0] if active == 'green' else [0, 0.18, 0], [0, 0.8, 0] if active == 'green' else [0, 0, 0])

while robot.step(timestep) != -1:
    t = robot.getTime() % sum(duration for _, duration in states)
    acc = 0
    for state, duration in states:
        if acc <= t < acc + duration:
            set_color(state)
            break
        acc += duration
