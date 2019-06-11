# -*- coding: utf-8 -*-

"""GA.py

遗传算法类
"""

import random
from Life import Life


class GA(object):

    def __init__(self, x_rate=0.7, mutation_rate=0.005, life_count=50, gene_length=100, judge=lambda lf, av: 1,
                 save=lambda: 1, mk_life=lambda: None, x_func=None, m_func=None):
        self.x_rate = x_rate
        self.mutation_rate = mutation_rate
        self.mutation_count = 0
        self.generation = 0
        self.lives = []
        self.bounds = 0.0  # 得分总数
        self.best = None
        self.life_count = life_count
        self.gene_length = gene_length
        self.__judge = judge
        self.save = save
        self.mk_life = mk_life  # 默认的产生生命的函数
        self.x_func = (x_func, self.__x_func)[x_func == None]  # 自定义交叉函数
        self.m_func = (m_func, self.__m_func)[m_func == None]  # 自定义变异函数

        for i in range(life_count):
            self.lives.append(Life(self, self.mk_life()))

    def __x_func(self, p1, p2):
        # 默认交叉函数
        r = random.randint(0, self.gene_length)
        gene = p1.gene[0:r] + p2.gene[r:]
        return gene

    def __m_func(self, gene):
        # 默认突变函数
        r = random.randint(0, self.gene_length - 1)
        gene = gene[:r] + ("0", "1")[gene[r:r] == "1"] + gene[r + 1:]
        return gene

    def __bear(self, p1, p2):
        # 根据父母 p1, p2 生成一个后代
        r = random.random()
        if r < self.x_rate:
            # 交叉
            gene = self.x_func(p1, p2)
        else:
            gene = p1.gene

        r = random.random()
        if r < self.mutation_rate:
            # 突变
            gene = self.m_func(gene)
            self.mutation_count += 1

        return Life(self, gene)

    def __get_one(self):
        # 根据得分情况，随机取得一个个体，机率正比于个体的score属性
        r = random.uniform(0, self.bounds)
        for lf in self.lives:
            r -= lf.score;
            if r <= 0:
                return lf

    def __new_child(self):
        # 产生新的后代
        return self.__bear(self.__get_one(), self.__get_one())

    def judge(self, f=lambda lf, av: 1):
        # 根据传入的方法 f ，计算每个个体的得分
        last_avg = self.bounds / float(self.life_count)
        self.bounds = 0.0
        self.best = Life(self)
        self.best.set_score(-1.0)
        for lf in self.lives:
            lf.score = f(lf, last_avg)
            if lf.score > self.best.score:
                self.best = lf
            self.bounds += lf.score

    def next(self, n=1):
        # 演化至下n代
        while n > 0:
            # self.__getBounds()
            self.judge(self.__judge)
            new_lives = [Life(self, self.best.gene)]
            # self.bestHistory.append(self.best)
            while len(new_lives) < self.life_count:
                new_lives.append(self.__new_child())
            self.lives = new_lives
            self.generation += 1
            # print("gen: %d, mutation: %d, best: %f" % (self.generation, self.mutationCount, self.best.score))
            self.save(self.best, self.generation)

            n -= 1
