import pygame
import numpy as np

# ===== PHYSICS CONSTANTS =====
G = 6.67430e-11 # Gravitational constant (m^3 / kg / s^2)
SCALE = 1.6e-10 
TIME_STEP = 60 * 60 * 12

# ===== DISPLAY =====
WIDTH, HEIGHT = 1920, 1080
FPS = 60

# ===== INITIALIZE DICTIONARIES =====
planet_trackers = {}

# ===== YOSHIDA COEFFICIENTS =====
YOSHIDA_W1 = 1 / (2 - 2**(1/3))
YOSHIDA_W2 = -2**(1/3) * YOSHIDA_W1
YOSHIDA_WEIGHTS = [YOSHIDA_W1, YOSHIDA_W2, YOSHIDA_W1]

# ===== PYGAME INITIALIZATION =====
pygame.init()
font = pygame.font.SysFont("Comic Sans", 14)
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Solar System Simulator")
clock = pygame.time.Clock()

# ===== BODY CLASS =====
class Body:
    def __init__(self, name, x, y, mass, radius, color, velocity):
        self.name = name
        self.position = np.array([x, y], dtype = 'float64') # meters
        self.velocity = np.array(velocity, dtype = 'float64') # m/s
        self.acceleration = np.array([0.0, 0.0], dtype = 'float64') # m/s^2
        self.mass = mass
        self.radius = radius
        self.color = color
        self.path = []
                
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
                for pos in self.path[-3000:]
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

# ===== VELOCITY CALCULATOR =====
def orbital_velocity(sun_mass, a, r):
    return np.sqrt(G * sun_mass * (2 / r - 1 / a)) # uses the vis-visa equation

# ===== CREATE PLANET FUNCTION =====
def create_planet(name, data):
    a = data["a"] # semi-major axis
    e = data["e"] # eccentricity
    r = a * (1 - e) # perihelion distance
    
    v = orbital_velocity(sun.mass, a, r)
    
    return Body(
        name = name,
        x = r,
        y = 0,
        mass = data["mass"],
        radius = data["radius"],
        color = data["color"],
        velocity = [0, v] # tangential velocity at perihelion
    )

# ===== REVOLUTION AND ENERGY TRACKER FUNCTION =====
def track_revolutions_and_energy(body, sim_time):
    tracker = planet_trackers[body.name]
    
    # angle tracking
    angle = np.arctan2(body.position[1], body.position[0])
    delta_angle = angle - tracker["prev_angle"]
    
    # wrap around -pi to pi
    if delta_angle < -np.pi:
        delta_angle += 2 * np.pi
    elif delta_angle > np.pi:
        delta_angle -= 2 * np.pi
        
        
    tracker["angle_total"] += delta_angle
    tracker["prev_angle"] = angle
    
    # revolution complete
    if tracker["angle_total"] >= 2 * np.pi:
        tracker["revolutions"] += 1
        days_pased = sim_time / (60 * 60 * 24)
        print(f"{body.name} completed {tracker['revolutions']} revolution(s) in {days_pased:.2f} days!")
        tracker["angle_total"] = 0
        
    # energy tracking 
    days_now = sim_time / (60 * 60 * 24)
    # every 100 days the kinetic energy will be outputted
    if int(days_now) >= tracker["last_energy_day"] + 100:
        kinetic = 0.5 * body.mass * np.linalg.norm(body.velocity) ** 2
        print(f"[Day {int(days_now)}] {body.name} KE: {kinetic:.2e} J")
        tracker["last_energy_day"] = int(days_now)

# ===== TRACKER INITIALIZATION =====
def init_tracker(body):
    planet_trackers[body.name] = {
        "prev_angle": np.arctan2(body.position[1], body.position[0]),
        "angle_total": 0,
        "revolutions": 0,
        "last_energy_day": 0
    }

# ===== BODY DATA =====
# mass in in kg, radius is in m, and velocity is in m/s^2 
sun = Body(
    name = "Sun",
    x = 0,
    y = 0,
    mass = 1.989e30,
    radius = 6.9634e8,
    color = (255, 255, 0),
    velocity = [0, 0]
)

planet_data = {
    "Mercury": {"mass": 3.3e23, "radius": 2.4e6, "color": (169, 169, 169), "a": 5.791e10, "e": 0.2056},
    "Venus":   {"mass": 4.87e24, "radius": 6.1e6, "color": (218, 165, 32), "a": 1.082e11, "e": 0.0068},
    "Earth":   {"mass": 5.972e24, "radius": 6.371e6, "color": (0, 102, 204), "a": 1.496e11, "e": 0.0167},
    "Mars":    {"mass": 6.417e23, "radius": 3.389e6, "color": (255, 69, 0), "a": 2.279e11, "e": 0.0934},
    "Jupiter": {"mass": 1.9e27, "radius": 7.1e7, "color": (218, 165, 105), "a": 7.785e11, "e": 0.0489},
    "Saturn":  {"mass": 5.7e26, "radius": 6.0e7, "color": (210, 180, 140), "a": 1.433e12, "e": 0.0565},
    "Uranus":  {"mass": 8.7e25, "radius": 2.7e7, "color": (173, 216, 230), "a": 2.877e12, "e": 0.0457},
    "Neptune": {"mass": 1.0e26, "radius": 2.4e7, "color": (0, 0, 250), "a": 4.503e12, "e": 0.0113}
}


# ===== INITIALIZE BODIES =====
bodies = [sun]


for name, data in planet_data.items():
    planet = create_planet(name, data)
    bodies.append(planet) # puts the planets into the bodies dictionary
    init_tracker(planet) # creates a tracker for each planet
    
    
# ===== MAIN LOOP =====
running = True
simulation_time = 0

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
    
    # ----- YOSHIDA 4TH ORDER SYMPLETIC INTEGRATION -----
    for weight in YOSHIDA_WEIGHTS:
        delta_t = TIME_STEP * weight
        forces = {body.name: np.zeros(2) for body in bodies} # initailizes all forces
    
        # step 1: compute forces at current position
        for i in range(len(bodies)):
            for j in range(i + 1, len(bodies)):
                body_i = bodies[i]
                body_j = bodies[j]
                # compute the gravitational force exerted by the current body with respect to one of the planets
                r_vec = body_j.position - body_i.position # calcualtes the displacement vector from the planet to the current body, basically the position of the current body that is relative to the planet in x/y coors
                r_mag = np.linalg.norm(r_vec) # calculates the magnitude of the displacement vector above, basically the actually distance of the current body and the planet
                if r_mag == 0:
                    continue
                
                force_mag = G * (body_i.mass * body_j.mass) / (r_mag **2) # newton's law of universal gravitation
                force_dir = r_vec / r_mag # calculates the unit vector of the displacement vector, basically the direction
                force = force_dir * force_mag # converts the scalar forces into a vector force, which have magnitude and direction
                
                forces[body_i.name] += force 
                forces[body_j.name] -= force
            
        # step 2: first half-step velocity update
        for body in bodies:
            body.velocity += 0.5 * forces[body.name] / body.mass * delta_t
        
        # step 3: full position update
        for body in bodies:   
            body.position += body.velocity * delta_t
            body.path.append(body.position.copy())
            
        # step 4: recompute forecs after move
        for i in range(len(bodies)):
            for j in range(i + 1,len(bodies)):
                body_i = bodies[i]
                body_j = bodies[j]
                # compute the gravitational force exerted by the current body with respect to one of the planets
                r_vec = body_j.position - body_i.position # calcualtes the displacement vector from the planet to the current body, basically the position of the current body that is relative to the planet in x/y coors
                r_mag = np.linalg.norm(r_vec) # calculates the magnitude of the displacement vector above, basically the actually distance of the current body and the planet
                if r_mag == 0:
                    continue
                
                force_mag = G * (body_i.mass * body_j.mass) / (r_mag **2) # newton's law of universal gravitation
                force_dir = r_vec / r_mag # calculates the unit vector of the displacement vector, basically the direction
                force = force_dir * force_mag # converts the scalar forces into a vector force, which have magnitude and direction
                
                forces[body_i.name] += force 
                forces[body_j.name] -= force
            
        # step 5: second half-step velocity update
        for body in bodies:
            body.velocity += 0.5 * forces[body.name] / body.mass * delta_t
    
        
    # Track simulation time
    simulation_time += TIME_STEP
    
    # Output the revolution and energy
    for body in bodies:
        if body.name != "Sun":
            track_revolutions_and_energy(body, simulation_time)
    

        
    screen.fill((0, 0, 10)) # space block
    for body in bodies:
        body.draw(screen)
        
    pygame.display.flip()
    
pygame.quit()