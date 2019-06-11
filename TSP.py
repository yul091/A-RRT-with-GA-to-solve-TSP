# -*- coding: utf-8 -*-

"""TSP.py

TSP genetic algorithm
"""

import sys
import random
import math
import time
import tkinter
import threading
from functools import reduce
from GA import GA


class MyTSP(object):
    """TSP"""

    def __init__(self, root, width=800, height=600, n=20, starter=(400,300)):
        # width & height define the size of the map, n means the number of targets(contain starting and ending point), starter is the coordinate of the starting point
        self.root = root
        self.width = width
        self.height = height
        self.n = n
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
            x = random.random() * (self.width - 60) + 30
            y = random.random() * (self.height - 60) + 30
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
            distance += math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        #print('distance: ',distance)
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
        self.__lock.acquire()
        self.__running = True
        self.__lock.release()

        while self.__running:
            self.ga.next()
            self.line(self.ga.best.gene)
            self.title("TSP - gen: {}".format(self.ga.generation))
            self.canvas.update()

        self.__t = None

    def line(self, order):
        """connect all the nodes in order"""
        self.canvas.delete("line")

        def line2(i1, i2):
            p1, p2 = self.nodes[i1], self.nodes[i2]
            self.canvas.create_line(p1, p2, fill="#000000", tags="line")
            return i2

        reduce(line2, order, order[-1])

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
