import numpy as np

from twodanim import twodanimation

def vector(x, y):
    return np.array([x,y], np.float64)
    

class Update(object):
    def update(self, T, deltaT):
        pass
    
    
class Force(Update):
    def calculate(self, x, v, mass):
        return vector(0,0)
    
class Wall(Update):
    def calculate(self, x, v, mass):
        return x, v

    
    
class System(object):
    def __init__(self, maxT, deltaT, 
                     figsize=(5,5),
                     xlimits=(0,20), 
                     ylimits=(-10,10)):
        self._maxT = maxT
        self._deltaT = deltaT
        self._forces = []
        self._walls = []
        
        self._particles = []
        
        # for the simulation
        self._figsize = figsize
        self._xlimits = xlimits
        self._ylimits = ylimits
        
        self._system_velocity = SystemVelocity(maxT, deltaT)
        
        
    def add_particle(self, p):
        self._particles.append(p)
        
    def add_force(self, f):
        self._forces.append(f)
        
    def add_wall(self, w):
        self._walls.append(w)
        
    def additional_graphics(self, ax):
        pass
    
    def analyze(self, T):
        # particles send to class "SystemVelocity" function "analyze"
        self._system_velocity.analyze(T, self._particles)


    def simulate(self):
        # at this point the number of particles
        # are set
        
        anim = twodanimation(nrobjects=len(self._particles), figsize=self._figsize,
                                 xlimits=self._xlimits, ylimits=self._ylimits, callback=self.additional_graphics)
        
        
        for p in self._particles:
            p.set_forces(self._forces)
            p.set_walls(self._walls)
            
        T = 0
        while T < self._maxT:
            
            for p in self._particles:
                p.update(T, self._deltaT)
            for f in self._forces:
                f.update(T, self._deltaT)
            for w in self._walls:
                w.update(T, self._deltaT)
                              
            positions = [p.get_position() for p in self._particles]
            anim.append(T, positions)
            
            # room for analyzing data
            self.analyze(T)
            
            T += self._deltaT
            
        return anim.animation(int(T//self._deltaT))
    


class Particle(Update):
    def __init__(self, x, v, mass):
        self._x = x
        self._v = v
        self._mass = mass
        self._forces = []
        self._walls = []
    
    def set_forces(self, forces):
        self._forces = forces
        
    def set_walls(self, walls):
        self._walls = walls
        
    def get_position(self):
        return self._x
    
    def get_velocity(self):
        #print(self._v)
        return self._v
    
    def update(self, T, deltaT):
        # check foe the collision
        for wall in self._walls:
            self._x, self._v = wall.calculate(self._x, self._v, self._mass)
        
        # Calculate the total force acting on the particle
        total_force = vector(0, 0)
        for force in self._forces:
            total_force += force.calculate(self._x, self._v, self._mass)
        
        # Calculate the acceleration based on the total force
        acceleration = total_force / self._mass
        
        # Update the velocity based on the acceleration and time step
        self._v += acceleration * deltaT
        
        # Update the position based on the velocity and time step
        self._x += self._v * deltaT
        
        
        
        
        
##################### Gravitational force #######################
# classs for force (gravity) apply 
class Gravity(Force):
    def __init__(self, gravity):
        self.gravity = gravity
    
    def calculate(self, x, v, mass):
        # Newton's 2nd law: F = ma
        # here, the x-component of force is zero, because gravity does not have
        # any y component
        return vector(0, mass * self.gravity)



    
    
#################### Boundary ##########################
# set the class for walls and floors
class Floor(Wall):
    
    # arguments: down floor, top floor, right wall and left wall
    def __init__(self, down_floor, up_floor, right_wall, left_wall ):
        self.down_floor = down_floor
        self.up_floor = up_floor
        self.right_wall = right_wall
        self.left_wall = left_wall

    # define the calculate the function to reverse the velocity during
    # floor and wall collisions
    def calculate(self, x, v, mass):
        
        if x[1] <= self.down_floor:
            # Reverse the y-component of velocity during down floor collision
            v[1] = -v[1]
            return x, v
        
        elif x[1] > self.up_floor:
            # Reverse the y-component of velocity during the top floor collision
            v[1] = -v[1]  
            return x, v
        
        # "if" is used here inspite of "elif" to reverse the velocities
        # during corner collision (up/down floor and right/left wall)
        if x[0] > self.right_wall:
            # Reverse the x-component of velocity during the right wall collision 
            v[0] = -v[0]
            return x, v
        
        elif x[0] <= self.left_wall:
            # Reverse the x-component of velocity during the left wall collision 
            v[0] = -v[0]
            return x, v        
        
        # if there is no collision happened, then return the original value
        return x, v
    


################## Analysis ##########################

# Analysis of the velocity of the particle    
class SystemVelocity(object):
    def __init__(self, maxT, deltaT):
        self._maxT = maxT
        self._deltaT = deltaT
        self._times = np.arange(0, maxT, deltaT)
        self._velocities = np.zeros_like(self._times)

    # calculate the total velocity from x and y component of velocity
    def analyze(self, T, particles):
        velocity = np.linalg.norm(particles[0].get_velocity())
        # extract the index to append the velocities
        index = int(T / self._deltaT)
        # append the velocities to "self._velocities"
        self._velocities[index] = velocity  # Record velocity
        
    # combine the velocities and time for ploting
    def get_speed_data(self):
        return np.column_stack((self._times, self._velocities))