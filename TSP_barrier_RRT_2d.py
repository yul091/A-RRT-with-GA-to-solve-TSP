# -*- coding: utf-8 -*-
"""
Created on Sat Jun  8 11:26:34 2019

@author: YUFEI
"""

import sys
import random
import math
import time
import tkinter
import threading
from functools import reduce
from GA import GA
import numpy as np
import matplotlib.pyplot as plt; plt.ion()
import matplotlib; matplotlib.use("TKAgg")
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import RRT_2d as RRT

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  
def load_map(fname):
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'xmax', 'ymax', 'r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = np.array(mapdata[~blockIdx][['xmin', 'ymin', 'xmax', 'ymax', 'r', 'g', 'b']].tolist())
  blocks = np.array(mapdata[blockIdx][['xmin', 'ymin', 'xmax', 'ymax', 'r', 'g', 'b']].tolist())
  return boundary, blocks


def runtest(mapfile, start, goal, verbose = True):
  boundary, blocks = load_map(mapfile)
  # Main loop
  robotpos = np.copy(start)
  poslist = [robotpos]
  numofmoves = 0
  success = True
  tryt = 1
  starttime = tic()
  while True:
    # Call the robot planner
    if tryt == 1:
      t0 = tic()
      RT = RRT.RobotPlanner(boundary, blocks, start, goal, seg = 0.01)
      movetime = max(1, np.ceil((tic() - t0) / 2.0))
      newrobotpos = robotpos
      print('move time: %d' % movetime)
      tryt += 1
      if movetime > 2:
        success = False
        print('ERROR: Initial two long\n')
        break
    # See if the planner was done on time
    t0 = tic()
    newrobotpos = RT.rrt()
    movetime = max(1, np.ceil((tic()-t0)/2.0))
    # print('move time: %d' % movetime)
    # See if the planner was done on time
    if movetime > 2:
      newrobotpos = robotpos-0.5 + np.random.rand(3)

    # Make the move
    robotpos = newrobotpos
    poslist.append(robotpos)
    numofmoves += 1
    # Check if the goal is reached
    if sum((robotpos-goal)**2) <= 0.1:
      break
  endtime = tic()
  print('time: ', endtime - starttime)
  poslist = np.array(poslist)
  return poslist, numofmoves


class MyTSP(object):
    """TSP"""

    def __init__(self, root, width=700, height=500, n=2, starter=(280,250)): 
        #width & height define the size of the map, n means the number of targets(contain starting and ending point), starter is the coordinate of the starting point
        self.root = root
        self.width = width
        self.height = height
        self.n = n
        #self.barrier = [(310,100,390,350),(10,200,70,480),(150,20,240,300),(450,220,580,450),(630,0,690,260),(180,400,370,450),(450,50,570,180)] 
        #self.barrier = [(310,100,390,220),(310,260,390,350),(70,50,110,180),(70,280,100,480),(150,80,240,210),(150,250,240,300),(450,220,580,450),(630,0,690,260),(180,400,370,450),(450,50,570,180)] 
        #self.barrier = [(310,100,390,220),(310,260,390,350),(70,50,110,180),(70,280,100,480),(150,250,240,300),(630,0,690,260),(180,400,370,450),(450,50,570,180)]
        #self.barrier = [(310,100,390,220),(70,50,110,180),(70,280,100,480),(150,250,240,300),(630,0,690,260),(180,400,370,450),(450,50,570,180)]  
        #self.barrier = [(310,100,390,220),(70,50,110,180),(70,280,100,480),(630,0,690,260),(180,400,370,450),(450,50,570,180)]
        self.barrier = [(70,50,110,180),(150,250,240,300),(630,0,690,260),(180,400,370,450),(450,50,570,180)]
        #self.barrier = [(70,50,110,180),(630,0,690,260),(310,100,390,220)]
        #self.barrier = [(70,50,110,180),(310,100,390,220)]
        #define the rectange coordinate of barriers
        self.canvas = tkinter.Canvas(
            root,
            width=self.width,
            height=self.height,
            bg="#ffffff",
            xscrollincrement=1,
            yscrollincrement=1
        )
        self.canvas.pack(expand=tkinter.YES, fill=tkinter.BOTH)
        self.title("TSP path planning")
        self.__r = 5
        self.__t = None
        self.__lock = threading.RLock()
        self.__running = False
        self.nodes = []
        self.nodes2 = []
        self.ga = None
        self.order = []
        self.starter = starter #define the starting and ending point
        self.__bind_events()
        self.new()
        

    def __bind_events(self):
        self.root.bind("q", self.quite) #quit the program
        self.root.bind("n", self.new) #initialize new evolvement
        self.root.bind("e", self.evolve) #begin evolvement
        self.root.bind("s", self.stop) #stop evolvement


    def title(self, s):
        self.root.title(s)

    def new(self, evt=None):
        self.__lock.acquire()
        self.__running = False
        self.__lock.release()

        self.clear()
        self.nodes = []  # 节点坐标
        self.nodes.append(self.starter)
        
        for i in range(len(self.barrier)):
            self.canvas.create_rectangle(self.barrier[i][0],self.barrier[i][1],self.barrier[i][2],self.barrier[i][3],fill='yellow')
            
        node = self.canvas.create_oval(
                self.starter[0] - self.__r,
                self.starter[1] - self.__r, self.starter[0] + self.__r, self.starter[1] + self.__r,
                fill="#000000",
                outline="#000000",
                tags="node",
            )
        self.nodes2 = []  # 节点图片对象
        self.nodes2.append(node)
        for i in range(self.n):
            right_time = 0
            while right_time == 0:
                x = random.random() * (self.width - 60) + 30
                y = random.random() * (self.height - 60) + 30
                error_time = 0
                for j in range(len(self.barrier)):
                    if self.barrier[j][0]-40 <= x <= self.barrier[j][2]+40 and self.barrier[j][1]-40 <= y <= self.barrier[j][3]+40:
                        error_time += 1
                if error_time == 0:
                    break   
                   
            self.nodes.append((x, y))
            node = self.canvas.create_oval(
                x - self.__r,
                y - self.__r, x + self.__r, y + self.__r,
                fill="#ff0000",
                outline="#ff0000",
                tags="node",
            )
            self.nodes2.append(node)

        self.ga = GA(
            life_count=50,
            mutation_rate=0.09,
            judge=self.judge(),
            mk_life=self.mk_life(),
            x_func=self.x_func(),
            m_func=self.m_func(),
            save=self.save()
        ) #GA contains the genetic algorithm's parameters
        self.order = range(self.n + 1)
        self.line(self.order)

    def distance(self, order):
        """the sum of length of the connection lines of all nodes"""
        distance = 0
        for i in range(-1, self.n):
            i1, i2 = order[i], order[i + 1]
            p1, p2 = self.nodes[i1], self.nodes[i2]
            q1, q2 = tuple(np.array([2*p1[0],self.height])-np.array(p1)), tuple(np.array([2*p2[0],self.height])-np.array(p2))
            #print(p1,p2,q1,q2)
            path, len_path = runtest('./2Dbuilding.txt', np.array(q1), np.array(q2), True)
            path = list(path)
            path.append(np.array(q1))
            path.insert(0,np.array(q2))
            path.reverse()
            print(path)
            path_return = []
            for j in range(len(path)):
                path_return.append(np.array([2*path[j][0],self.height])-path[j])
            distance += len_path
        print('distance: ',distance)
        return distance

    def mk_life(self):
        def f():
            lst = [i for i in range(self.n + 1)]
            random.shuffle(lst)
            return lst

        return f

    def judge(self):
        """evaluation fitness function"""
        return lambda lf, av=100: 1.0 / self.distance(lf.gene)

    def x_func(self):
        """crossover function"""

        def f(lf1, lf2):
            p1 = random.randint(0, self.n)
            p2 = random.randint(self.n, self.n + 1)
            g1 = lf2.gene[p1:p2] + lf1.gene #child get gene crossover genes from parents
            # g2 = lf1.gene[p1:p2] + lf2.gene
            g11 = []
            for i in g1:
                if i not in g11:
                    g11.append(i) #no repetation
            return g11

        return f

    def m_func(self):
        """mutation function"""

        def f(gene):
            p1 = random.randint(0, self.n - 1)
            p2 = random.randint(self.n - 1, self.n)
            gene[p1], gene[p2] = gene[p2], gene[p1] #gene interchange
            return gene

        return f

    def save(self):
        def f(lf, gen):
            pass

        return f

    def evolve(self, evt=None):
        #tic()
        self.__lock.acquire()
        self.__running = True
        self.__lock.release()

        while self.__running:
            self.ga.next()
            self.line(self.ga.best.gene)
            self.title("TSP - gen: {}".format(self.ga.generation))
            self.canvas.update()

        self.__t = None
        #toc()

    def line(self, order):
        """connect all the nodes in order"""
        self.canvas.delete("line")
        for i in range(-1, self.n):
            i1, i2 = order[i], order[i + 1]
            p1, p2 = self.nodes[i1], self.nodes[i2]
            q1, q2 = tuple(np.array([2*p1[0],self.height])-np.array(p1)), tuple(np.array([2*p2[0],self.height])-np.array(p2))
            #print(p1,p2,q1,q2)
            path,len_path = runtest('./2Dbuilding.txt', np.array(q1), np.array(q2), True)
            path = list(path)
            path.append(np.array(q2))
            path.insert(0,np.array(q1))
            path.reverse()
            for i in range(len(path)):
                path[i] = tuple(path[i])
            #print(path)
            path_return = []
            for j in range(len(path)):
                path_return.append(np.array([2*path[j][0],self.height])-path[j])
            #print(path_return)
            for j in range(len(path)-1):
                c1, c2 = path_return[j], path_return[j+1]
                self.canvas.create_line(c1[0], c1[1], c2[0], c2[1], fill="#000000", tags="line")

    def clear(self):
        for item in self.canvas.find_all():
            self.canvas.delete(item)

    def quite(self, evt):
        self.__lock.acquire()
        self.__running = False
        self.__lock.release()
        sys.exit()

    def stop(self, evt):
        self.__lock.acquire()
        self.__running = False
        self.__lock.release()

    def mainloop(self):
        self.root.mainloop()


if __name__ == "__main__":
    MyTSP(tkinter.Tk()).mainloop()
