import math

class MotionPlanner:
    def __init__(self, v, a):
        # These parameters are fixed
        self.v = v
        self.a = a

    def trapezoidal_velocity_profile(self, start, goal):
        self.h = abs(start - goal)
        self.ratio = pow(self.v, 2) / self.a
        if (self.h < self.ratio) : # Full trapezoidal not possible => Triangular profile
            self.t = 2 * math.sqrt(self.h / self.a)
        else: # Full trapezoidal is possible
            self.t = 2 * (self.v / self.a) + (self.h - self.ratio) / self.v 
        return self.t
    
# Have a test
if __name__ == "__main__":
    
    motion_planner = MotionPlanner(700, 900)
    t = motion_planner.trapezoidal_velocity_profile(0, 1500)
    print(f"The time taken is: {t} seconds")