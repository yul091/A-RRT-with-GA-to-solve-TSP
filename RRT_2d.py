# -*- coding: utf-8 -*-
"""
Created on Fri Jun  7 23:46:08 2019

@author: YUFEI
"""

import numpy as np
from sklearn.neighbors import NearestNeighbors
import matplotlib;matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import time

def load_map(fname):
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'xmax', 'ymax', 'r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = np.array(mapdata[~blockIdx][['xmin', 'ymin', 'xmax', 'ymax','r','g','b']].tolist())
  blocks = np.array(mapdata[blockIdx][['xmin', 'ymin', 'xmax', 'ymax', 'r','g','b']].tolist())
  return boundary, blocks
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))


class RobotPlanner:

  def __init__(self, boundary, blocks, start, goal, epsilon = 50, seg = 0.1):
    self.boundary = np.round(boundary, 1)
    self.blocks = np.round(blocks, 1)
    self.nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto')
    self.epsilon = epsilon
    self.seg = seg
    self.start_con = start
    self.goal = goal
    self.goa2st = [goal]
    self.V = [start]
    self.par = [0]
    self.loop = 0
    self.ind = -1
    self.len_goa2st = -10


  def sample(self, method = 'uniform'):
    if method == 'uniform':
      rndx = np.random.uniform(self.boundary[0, 0], self.boundary[0, 2])
      rndy = np.random.uniform(self.boundary[0, 1], self.boundary[0, 3])
      #rndz = np.random.uniform(self.boundary[0, 2], self.boundary[0, 5])
      return np.array((rndx,rndy))
    elif method == 'gauss':
      rnd = self.boundary[2:4] - self.boundary[:2]
      xy = np.random.multivariate_normal([0,0], rnd*np.eye(2))
      return xy

  def nearest(self, tree, target):
    '''

    :param tree: rrt tree list
    :param target:
    :return: nearest node's index in the list
    '''
    self.nbrs.fit(tree)
    tarind = self.nbrs.kneighbors(np.array(target).reshape(1,2), n_neighbors=1, return_distance=False)[0][0]
    return tarind

  def dist(self, pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

  def segline(self, pos1, pos2):
    dis = self.dist(pos1, pos2)
    segn = int(np.ceil(dis / self.seg))
    segns = np.linspace(pos1, pos2, segn)
    return segns

  def node_collision(self, newrp):
    '''
    valid = True, not collision; valid = False is collison
    '''
    valid = True
    if (newrp[0] <= self.boundary[0, 0] or newrp[0] >= self.boundary[0, 2] or \
            newrp[1] <= self.boundary[0, 1] or newrp[1] >= self.boundary[0, 3] ):
      valid = False

    for k in range(self.blocks.shape[0]):
      if (newrp[0] >= self.blocks[k, 0] and newrp[0] <= self.blocks[k, 2] and \
              newrp[1] >= self.blocks[k, 1] and newrp[1] <= self.blocks[k, 3] ):
        valid = False
    return valid

  def edge_collision(self, pos1, pos2):
    '''
    segment the line between pos1 and pos2 in segn pieces, the length in each piece is smaller than
    the smallest size of blocks(0.1)
    if all the segmented nodes are not in collision with blocks, then this path is not collision.
    There is a situation this method invalid: the collision happens at the corner of block;
    Nevertheless, all the blocks in maps have their edge larger than 0.1, this method can still give a decent result
    '''
    segns = self.segline(pos1, pos2)
    for node in segns:
      if not self.node_collision(node):
        return False
    return True

  def steer(self, near, rand):
    dist = self.dist(near, rand)
    if dist <= self.epsilon:
      new = rand
    else:
      ratio = self.epsilon/dist
      new = near * (1 - ratio) + rand * ratio
    # check collision
    if not self.edge_collision(new, near):
      segns = self.segline(near, new)
      i = 0
      while self.node_collision(segns[i,:]):
        i += 1
      i -= 1
      new = segns[i, :]

    return new

#  def plot_tree(self, V, par):
#
#    # plot setting
#    unit_sizes = np.zeros(len(par))
#    linewidths = 5 * unit_sizes + 0.1
#    node_sizes = linewidths ** 2
#
#    # edges
#    edges = np.array([[loc, V[par]] for loc, par in zip(V, par)])
#    fig = plt.figure()
#    ax = fig.add_subplot(111, projection='3d') #
#    # plot edges
#    ax.add_collection3d(Line3DCollection(edges, linewidths=1))
#    # plot nodes
#    ax.scatter(*list(np.array(V).transpose()), s=node_sizes, c=(0, 0))
#    ax.set_title('Tree')
#    ax.set_xlabel('X')
#    ax.set_ylabel('Y')
#    #ax.set_zlabel('Z')
#    ax.set_xlim(self.boundary[0,0], self.boundary[0,2])
#    ax.set_ylim(self.boundary[0,1], self.boundary[0,3])
#    #ax.set_zlim(self.boundary[0,2], self.boundary[0,5])
#    plt.tight_layout()
#    plt.show()

  def rrt(self):
    st = tic()

    while (self.goal.tolist() not in [lis.tolist() for lis in self.V]):
      rand = self.sample()
      nearind = self.nearest(self.V, rand)
      near = self.V[nearind]
      new = self.steer(near, rand)
      # ith node's parent has the index in par[i]
      self.V.append(new)
      self.par.append(nearind)

      et = tic()
      if (et - st) > 1.5:
        return self.start_con

      # find target
      if self.loop % 100 == 0:
        goalind = self.nearest(self.V, self.goal)
        goal1 = self.V[goalind]
        goal2 = self.steer(goal1, self.goal)
        if np.array_equal(goal2, self.goal):
          self.V.append(self.goal)
          self.par.append(goalind)
          #self.plot_tree(self.V, self.par)
          break
      self.loop += 1


    # generate path
    if self.len_goa2st == -10:
      self.ind = self.par[self.ind]
      while self.ind != 0:
        self.goa2st.append(self.V[self.ind])
        # check time
        et = tic()
        if (et - st) > 1.5:
          return self.start_con
        self.ind = self.par[self.ind]
      self.goa2st.append(self.start_con)
      self.len_goa2st = len(self.goa2st)
      return self.start_con
    else:
      self.len_goa2st -= 1
      return self.goa2st[self.len_goa2st]



if __name__ == "__main__":
  start = np.array([30, 30])
  goal = np.array([660, 450])
  boundary, blocks = load_map('./2Dbuilding.txt')
  rrt = RobotPlanner(boundary, blocks, start, goal)
  goa2st = rrt.rrt()






