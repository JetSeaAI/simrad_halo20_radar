#!/usr/bin/env python
# -*- coding: utf-8 -*-
from numpy import pi, sqrt, tan, cos, sign


# this module is modified from blueRobotics Ping 360 sonar driver to work with the Simrad Halo20 radar
# the original code can be found at https://github.com/JetSeaAI/ping360_sonar/blob/ros2/ping360_sonar/ping360_sonar/sonar_interface.py

class RadarInterface:

    def __init__(self):
        self.angle = 0

    def configureAngles(self, aperture_deg, step_deg, ensure_divisor):
        # to gradians
        target_half_aperture = int(aperture_deg*200/360+0.5)
        best_half_aperture = target_half_aperture
        self.angle_step = self.degrees2grad(step_deg)

        # ensure angle_step is a divisor of max-min in gradians, necessary for LaserScan messages
        if ensure_divisor:            
            # look around step, allow increased aperture
            target_step = self.angle_step
            
            # not too far from requested aperture, as close as possible to requested step (impacts turn duration)
            computeCost = lambda step,half_aperture: 1000 if half_aperture%step != 0 else abs(step-target_step) + abs(half_aperture-target_half_aperture)
            
            best_cost = computeCost(self.angle_step, target_half_aperture)
            if best_cost != 0:                
                for step in range(1, target_step*2):
                    for half_aperture in range(target_half_aperture, min(target_half_aperture+10, 200)+1):
                        cost = computeCost(step, half_aperture)
                        if cost < best_cost:
                            best_cost = cost
                            self.angle_step = step
                            best_half_aperture = half_aperture
                        
        self.angle_min = -best_half_aperture
        self.angle_max = best_half_aperture
        if self.angle_max == 200:                
            self.angle_max -= self.angle_step
        if self.angle < self.angle_min or self.angle > self.angle_max or (self.angle-self.angle_min) % self.angle_step != 0:
            self.angle = 0
    
    @staticmethod
    
    def grad2rad(grad):
        return grad*pi/200
    def rad2grad(self,rad):
        return rad*200/pi
    def degrees2grad(self, deg):
        return deg*200/180
    def degrees2rad(self, deg):
        return deg*pi/180
    def angleMin(self):
        return self.grad2rad(self.angle_min)
    def angleMax(self):
        return self.grad2rad(self.angle_max)
    def angleStep(self):
        return self.grad2rad(self.angle_step)
    def currentAngle(self):
        return self.grad2rad(self.angle)
    def angleCount(self):
        return (self.angle_max-self.angle_min)//self.angle_step
    def angleIndex(self):
        if self.angle_step > 0:
            return (self.angle-self.angle_min)//self.angle_step
        return (self.angle-self.angle_max)//self.angle_step
    
    def fullScan(self):
        return self.angle_min == -200
    

    
    def updateAngle(self,angle):

        self.angle=angle
        # self.angle += self.angle_step
        
        if self.angle_min == -200:
            # full scan
            end_turn = self.angle + self.angle_step > self.angle_max
            if self.angle > self.angle_max:
                self.angle = self.angle_min
            return end_turn
        
        # sector scan, check near end of sector
        if self.angle + self.angle_step >= self.angle_max or self.angle + self.angle_step <= self.angle_min:
            self.angle_step *= -1
            return True
        
        return False




# handles an angular sector of the image
class Bound:
    radius = 0
    def __init__(self, x, tm, tM):
        self.x = x
        if type(tM) == int:
            self.low = Bound.clamp(tm*x)
            self.up = int(tM*sqrt(Bound.radius**2-x**2-1))
        else:
            self.low = Bound.clamp(x*tm)
            self.up = Bound.clamp(x*tM)

            
            if self.up**2 + x**2 > Bound.radius**2:
                self.up = int(sign(self.up) * sqrt(Bound.radius**2-x**2-1))
                
        if self.up < self.low:
            self.low,self.up = self.up,self.low
            
    #staticmethod
    def clamp(coord):
        if coord < -Bound.radius+1:
            return -Bound.radius+1
        elif coord > Bound.radius-1:
            return Bound.radius-1
        return int(coord)
            
class Sector:
    def __init__(self):
        self.dr = None
        
    def configure(self, samples, radius):
        self.dr = radius/samples
        Bound.radius = radius
        
    def init(self, angle, step):
        angle_min = angle-step/2
        angle_max = angle+step/2
        xmin, xmax,same_side = self.xLimits(angle_min, angle_max)
        tm, tM = tan(angle_min), tan(angle_max)        
        self.bounds = []

        if same_side:
            # same side
            if abs(tm) > abs(tM):
                tm,tM = tM,tm
            for x in range(xmin, xmax+1):
                self.bounds.append(Bound(x,tm,tM))
        else:
            f = 1 if abs(angle-pi/2) < abs(angle+pi/2) else -1
            
            if f == -1:
                tm,tM = tM,tm
                
            for x in range(xmin, 0):
                self.bounds.append(Bound(x, tM,f))
            for x in range(0, xmax+1):
                self.bounds.append(Bound(x, tm,f))
                
        self.cur = -1
        
    def xLimits(self, angle_min, angle_max):
        cm = cos(angle_min)
        cM = cos(angle_max)
        if cM < cm:
            cm,cM = cM,cm
        if cm*cM > 0:
            if cM < 0:
                cM = 0
            else:
                cm = 0
        return Bound.clamp(round(Bound.radius*cm)), Bound.clamp(round(Bound.radius*cM)), cm*cM >= 0
    
    def nextPoint(self, x, y):
        if self.cur == -1:
            self.cur = 0
            x = self.bounds[0].x
            y = self.bounds[0].low
        elif y < self.bounds[self.cur].up:
            y += 1
        else:
            self.cur += 1
            if self.cur == len(self.bounds):
                return False, 0, 0, 0
            x = self.bounds[self.cur].x
            y = self.bounds[self.cur].low
        return True, x, y, int(round(sqrt(x*x+y*y)/self.dr))
