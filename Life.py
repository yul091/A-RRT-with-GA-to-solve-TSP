# -*- coding: utf-8 -*-

"""Life.py

生命类
"""

import random


class Life(object):

    def __init__(self, env, gene=None):
        self.env = env
        self.score = 0

        if gene is None:
            self.__rnd_gene()
        elif isinstance(gene, list):
            self.gene = []
            for k in gene:
                self.gene.append(k)
        else:
            self.gene = gene

    def __rnd_gene(self):
        self.gene = ""
        for i in range(self.env.gene_length):
            self.gene += str(random.randint(0, 1))

    def set_score(self, v):
        self.score = v

    def add_score(self, v):
        self.score += v
