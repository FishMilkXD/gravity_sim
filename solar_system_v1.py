import pygame
import numpy as np

# ===== PHYSICS CONSTANTS =====
G = 6.67430e-11 # Gravitational constant (m^3 / kg / s^2)
SCALE = 1.6e-10 
TIME_STEP = 60 * 60 * 12 # 1 day per simulation step

# ===== DISPLAY =====
WIDTH, HEIGHT = 1920, 1080
FPS = 60

pygame.init()
font = pygame.font.SysFont("Comic Sans", 14)
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Solar System Simulator")
clock = pygame.time.Clock()

# ===== BODY CLASS =====
class Body:
    def __init__(self, name, x, y, mass, radius_m, color, velocity):
        self.name = name
        self.position = np.array([x, y], dtype = 'float64') # meters
        self.velocity = np.array(velocity, dtype = 'float64') # m/s
        self.acceleration = np.array([0.0, 0.0], dtype = 'float64') # m/s^2
        self.mass = mass
        self.radius = radius_m
        self.color = color
        self.path = []
        
    def update(self, force, delta_time):
        global SCALE
        
        new_acceleration = force / self.mass
        
        # 1. update position using current velocity and acceleration
        self.position += self.velocity * delta_time + 0.5 * self.acceleration * delta_time ** 2

        # 2. update velocity using average of old and new acceleration 
        self.velocity += 0.5 * (self.acceleration + new_acceleration) * delta_time
                 
        # 3. store new acceleration for next iteration
        self.acceleration = new_acceleration
        
        # store acutal position (in meters)
        self.path.append(self.position.copy())
        
    def draw(self, screen):
        x = WIDTH // 2 + int(self.position[0] * SCALE)
        y = HEIGHT // 2 + int(self.position[1] * SCALE)
        r = max(2, int(self.radius * SCALE))
        
        # draws planet
        pygame.draw.circle(screen, self.color, (x, y), r)
        
        # draw orbit trail
        if len(self.path) > 2:
            trail = [
                (WIDTH // 2 + int(pos[0] * SCALE), HEIGHT // 2 + int(pos[1] * SCALE)) 
                for pos in self.path[-1000:]
            ]
            pygame.draw.lines(screen, self.color, False, trail, 1)
            
        # draw label 
        label = font.render(self.name, True, (255, 255, 255))
        screen.blit(label, (x + 5, y- 10))
        
        # HUD display
        time_step_days = TIME_STEP / (60 * 60 * 24)
        days_passed = simulation_time / (60 * 60 * 24)
        
        hud_text = f"Time Step: {time_step_days:.2f} days/frame | Days Passed: {int(days_passed)}"
        hud_render = font.render(hud_text, True, (255, 255, 255))
        screen.blit(hud_render, (20,20))
            
# ===== INIT BODIES =====
# x is the distance of the planet from the sun
sun = Body(
    name = "Sun",
    x = 0,
    y = 0,
    mass = 1.989e30,
    radius_m = 6.934e8,
    color = (255, 255, 0),
    velocity = [0,0]
)

mercury = Body(
    name = "Mercury",
    x = 5.8e10,
    y = 0,
    mass = 3.3e23,
    radius_m = 2.4e6,
    color = (169, 169, 169),
    velocity = [0, 47_870]
)

venus = Body(
    name = "Venus",
    x = 1.08e11,
    y = 0,
    mass = 4.87e24,
    radius_m = 6.1e6,
    color = (218, 165, 32),
    velocity = [0, 35_020]
)

earth = Body(
    name = "Earth",
    x = 1.496e11, # 149.6 million km
    y = 0,
    mass = 5.972e24,
    radius_m = 6.371e6,
    color = (0, 102, 204),
    velocity = [0, 29_783.89] # 29.78
)

mars = Body(
    name = "Mars",
    x = 2.27e11, # 227.9 million km
    y = 0,
    mass = 6.417e23,
    radius_m = 3.389e6,
    color = (255, 69, 0),
    velocity = [0, 29_077] 
)

jupiter = Body(
    name = "Jupiter",
    x = 7.78e11, 
    y = 0,
    mass = 1.9e27,
    radius_m = 7.1e7,
    color = (218, 165, 105),
    velocity = [0, 13_070] 
)

saturn = Body(
    name = "Saturn",
    x = 1.43e12, 
    y = 0,
    mass = 5.7e26,
    radius_m = 6.0e7,
    color = (210, 180, 140),
    velocity = [0, 9_690] 
)

uranus = Body(
    name = "Uranus",
    x = 2.87e12, 
    y = 0,
    mass = 8.7e25,
    radius_m = 2.7e7,
    color = (173, 216, 230),
    velocity = [0, 6_800] 
)

neptune = Body(
    name = "Neptune",
    x = 4.5e12, 
    y = 0,
    mass = 1.0e26,
    radius_m = 2.4e7,
    color = (0, 0, 250),
    velocity = [0, 5_430] 
)

bodies = [sun, earth]

# ===== MAIN LOOP =====
running = True
simulation_time = 0
revolutions = 0
prev_angle = np.arctan2(earth.position[1], earth.position[0])
while running:
    clock.tick(FPS)
    
    # ===== handle events =====
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
      
    
    keys = pygame.key.get_pressed()
    # ===== Handle Zoom =====
    if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]: # zoom out
        SCALE *= 1.1
    if keys[pygame.K_MINUS]: # Zoom in
        SCALE /= 1.1
    

          
    # ===== Phyiscs simulation =====
    for body in bodies:
        if body.name == "Sun":
            continue
        
        r_vec = sun.position - body.position
        r_mag = np.linalg.norm(r_vec)
        force_mag = G * sun.mass * body.mass / r_mag ** 2
        force_dir = r_vec / r_mag
        force = force_dir * force_mag
        body.update(force, TIME_STEP)
        
    # Track simulation time
    simulation_time += TIME_STEP

    # === Orbit angle tracking ===
    angle = np.arctan2(earth.position[1], earth.position[0])
    delta_angle = angle - prev_angle

    # Correct wrap-around across ±π
    if delta_angle < -np.pi:
        delta_angle += 2 * np.pi
    elif delta_angle > np.pi:
        delta_angle -= 2 * np.pi

    # Increment total revolution progress
    revolutions += delta_angle / (2 * np.pi)
    prev_angle = angle

    # Print when full orbit completes
    if revolutions >= 1:
        days = simulation_time / (60 * 60 * 24)
        print(f"✅ Earth completed one orbit in {days:.2f} days!")
        print(f"Testing velocity: {earth.velocity[1]} m/s")
        revolutions = 0  # reset if you want to track multiple orbits


    kinetic = 0.5 * earth.mass * np.linalg.norm(earth.velocity) ** 2
    potential = -G * sun.mass * earth.mass / np.linalg.norm(earth.position - sun.position)
    total_energy = kinetic + potential
    print(f"Day {simulation_time / 86400:.1f} | E = {total_energy:.2e} J")

        
    screen.fill((0, 0, 10)) # space block
    for body in bodies:
        body.draw(screen)
        
    pygame.display.flip()
    
pygame.quit()