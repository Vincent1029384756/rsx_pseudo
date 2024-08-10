import math
import numpy as np
import APF_func as af

class APF():

    def _init_(self, start, goal, obstacles):
        
        '''
        start: 3X1 np.array
        goal: 3X1 np.array
        obstacles: list of 3X1 np.arrays?
        '''

        #APF parameters
        '''
        zeta & d: APF parameters for calculating F_att and F_rep
        time_step: used for calculating new position
        '''
        self.zeta = 1.0
        self.time_step = 0.1 #sec
        self.d = 1.0
        self.v = np.array([0.0, 0.0, 0.0])
        self.k = 0
        self.cur_pos = start
        self.goal = goal
        self.obstacles = obstacles


    def change_zeta(self, zeta):
        self.zeta = zeta
    
    def change_time_step(self, time_step):
        self.time_step = time_step
    
    def change_d(self, d):
        self.d = d
    
    def move(self):
        self.cur_pos = self.cur_pos + self.v*self.time_step
    
    def calc_F_att(self):
        #calculate attractive force
        distance = np.linalg.norm(self.cur_pos, self.goal)

        if distance <= self.d:
            F_att = -self.zeta(self.cur_pos - self.goal)

        elif distance >= self.d:
            F_att = -self.d*self.zeta*(self.cur_pos - self.goal)

        return F_att
    
    def calc_rho(self):
        #rho is the distance from the point to the boundary
        rho, _ = af.shortest_dist(self.cur_pos, self.obstacles)

        return rho
    
    def cal_F_rep(self):
        #calculate repulsive force
        distance = np.linalg.norm(self.cur_pos, self.goal)

        if distance >= self.d:
            F_rep = 0

        elif distance <= self.d:
