import pygame
import numpy as np

# constants
WIDTH, HEIGHT = 1280, 1000
FPS = 60

# physics constants
G = 20 # artificial gravitational constant

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
running = True

# ===== define star and object =====
class Body:
    
    def __init__(self, x, y, mass, radius, color):
        self.position = np.array([x,y], dtype='float64')
        self.velocity = np.array([0,0], dtype='float64')
        self.mass = mass
        self.radius = radius
        self.color = color
        self.path = []
        
    def update(self, force, delta_t):
        acceleration = force / self.mass
        self.velocity += acceleration * delta_t
        self.position += self.velocity * delta_t
        self.path.append(tuple(self.position))
        
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position.astype(int), self.radius)
        
        if len(self.path) > 2:
            pygame.draw.lines(screen, self.color, False, self.path[-500:], 1)
            


star = Body(WIDTH // 2, HEIGHT // 2, mass = 20000, radius = 15, color = (255,255,0))


# ===== launch object with mouse =====
objects = []
launching = False
launch_start = None

while running:
    delta_t = clock.tick(FPS) / 1000 # delta time in seconds
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Begin drag
        if event.type == pygame.MOUSEBUTTONDOWN:
            launch_start = pygame.mouse.get_pos()
            launching = True
            
        # Launch Object
        if event.type == pygame.MOUSEBUTTONUP and launching:
            end = pygame.mouse.get_pos()
            dx = end[0] - launch_start[0]
            dy = end[1] - launch_start[1]
            velocity = np.array([dx, dy]) * 0.05 #scale down
            obj = Body(*launch_start, mass = 1, radius = 5, color = (0, 255, 255))
            obj.velocity = velocity
            objects.append(obj)
            launching = False
            
    screen.fill((0, 0, 0))
    star.draw(screen)
    
    for obj in objects:
        r_vector = star.position - obj.position
        r_magnitude = np.linalg.norm(r_vector)
        if r_magnitude > star.radius:
            force_magnitude = G * star.mass * obj.mass / r_magnitude**2
            force_direction = r_vector / r_magnitude
            force = force_direction * force_magnitude
            obj.update(force, delta_t)
        obj.draw(screen)
        
    pygame.display.flip()