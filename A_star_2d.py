# -*- coding: utf-8 -*-
"""
Created on Fri May 31 23:59:22 2019

@author: YUFEI
"""

import numpy as np
import time

class Node:

    def __init__(self, index, h, g=np.inf):
        self.x = index[0]
        self.y = index[1]
        #self.z = index[2]
        self.g = g
        self.h = h
        self.key = str(index)

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.h) + "," + str(self.g)

class RobotPlanner:

    def __init__(self, boundary, blocks, reso=10):
        '''
        :param boundary:
        :param blocks:
        :param reso: grid resolution
        '''
        self.boundary = boundary
        self.xmin = boundary[0, 0]
        self.ymin = boundary[0, 1]
        #self.zmin = boundary[0, 2]
        self.xwidth = int(round((boundary[0, 2] - boundary[0, 0]) / reso))
        self.ywidth = int(round((boundary[0, 3] - boundary[0, 1]) / reso))
        #self.zwidth = int(round((boundary[0, 5] - boundary[0, 2]) / reso))
        self.blocks = blocks
        self.reso = reso
        self.Parent = {}
        self.OPEN = {}
        self.CLOSED = {}
        self.find_goal = False
        # self.final_path = []

    def calc_heuristic(self, ind_start, ind_end):
        w = 1.0  # weight of heuristic
        # h = w * abs(ind_start - ind_end).sum()
        h = w * np.linalg.norm(ind_start - ind_end)
        return h

    def grid_index(self, point):
        x = point[0]
        y = point[1]
        #z = point[2]
        xind = int(round((x - self.xmin) / self.reso))
        yind = int(round((y - self.ymin) / self.reso))
        #zind = int(round((z - self.zmin) / self.reso))
        ind = np.array([xind, yind])
        return ind

    def NodeCheck(self, ind):
        n = self.blocks.shape[0]
        if (ind[0] < 0 or ind[0] > self.xwidth or \
            ind[1] < 0 or ind[1] > self.ywidth):
            return False
        for i in range(n):
            point1 = np.array([self.blocks[i, 0], self.blocks[i, 1]])
            point2 = np.array([self.blocks[i, 2], self.blocks[i, 3]])
            ind1 = self.grid_index(point1)
            ind2 = self.grid_index(point2)
            if (ind[0] >= ind1[0] and ind[0] <= ind2[0] and \
                    ind[1] >= ind1[1] and ind[1] <= ind2[1]):
                return False
        return True

    def find_final_path(self, nstart, ngoal, Parent):
        path = []
        path.append(np.array([ngoal.x*self.reso + self.xmin, ngoal.y*self.reso + self.ymin]))
        key = ngoal.key
        while (key != nstart.key):
            node = Parent[key]
            path.append(np.array([node.x*self.reso + self.xmin, node.y*self.reso + self.ymin]))
            key = node.key
        return path

    def Astar_Planner(self, start, goal):
        newrobotpos = np.copy(start)
        # begin = time.time()
        final_path = []
        # if self.find_goal:
            #if len(self.final_path) != 0:
                # newrobotpos = self.final_path.pop(-1)
            # else:
                # newrobotpos = goal * 1
        # else:
        ind_start = self.grid_index(start)
        ind_goal = self.grid_index(goal)
        nstart = Node(ind_start, self.calc_heuristic(ind_start, ind_goal), 0.0)
        ngoal = Node(ind_goal, self.calc_heuristic(ind_goal, ind_goal))
        [dX, dY] = np.meshgrid([-1, 0, 1], [-1, 0, 1])
        dR = np.vstack((dX.flatten(), dY.flatten()))
        dR = np.delete(dR, 4, axis=1)
        cost = np.sqrt(np.sum(dR ** 2, axis=0))
        # print(ngoal.key, nstart.key)
        self.OPEN[nstart.key] = nstart
        while True:
            # for k in range(200):
            c_id = min(self.OPEN, key=lambda o: self.OPEN[o].g + self.OPEN[o].h)
            #print(c_id)
            current = self.OPEN[c_id]
            #print(current)
            # print(current.key)
            if current.x == ngoal.x and current.y == ngoal.y:
                self.find_goal = True
                #print("Find goal")
                break

            # Remove the item from the open set
            del self.OPEN[c_id]
            # ADD it to the closed set
            self.CLOSED[c_id] = current

            temp = np.array([current.x, current.y])
            temp = temp[:, np.newaxis]
            inds = dR + temp

            for i in range(inds.shape[1]):
                ind = inds[:, i]
                if not self.NodeCheck(ind):
                    continue
                if str(ind) in self.CLOSED.keys():
                    continue
                next = Node(ind, self.calc_heuristic(ind, ind_goal))
                if next.g > current.g + cost[i]:
                    next.g = (current.g + cost[i]) * 1
                    self.OPEN[next.key] = next
                    self.Parent[next.key] = current

                # now = time.time()
                # if now - begin > 1:
                    # break
#            if self.find_goal:
                # self.final_path = self.find_final_path(nstart, ngoal, self.Parent)
        final_path = self.find_final_path(nstart, ngoal, self.Parent)
#            print(newrobotpos)
        return newrobotpos, final_path
