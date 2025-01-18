#!/usr/bin/env python3
import sys
import time
from enum import Enum
import numpy as np
import math
from kaiaai.util import OccupancyGrid2d


class FrontierCache():
  cache = {}

  def getPoint(self, x, y):
    idx = self.__cantorHash(x, y)

    if idx in self.cache:
      return self.cache[idx]

    self.cache[idx] = FrontierPoint(x, y)
    return self.cache[idx]

  def __cantorHash(self, x, y):
    return (((x + y) * (x + y + 1)) / 2) + y

  def clear(self):
    self.cache = {}

class FrontierPoint():
  def __init__(self, x, y):
    self.classification = 0
    self.mapX = x
    self.mapY = y


class WavefrontFrontierDetector():
  MapOpen = 1
  MapClosed = 2
  FrontierOpen = 4
  FrontierClosed = 8

  OCC_THRESHOLD = 10
  MIN_FRONTIER_SIZE = 5

  @staticmethod
  def getGoals(pose, map: OccupancyGrid2d):
    return WavefrontFrontierDetector.getFrontier(pose, map)

  @staticmethod
  def getNextGoal(pose, map: OccupancyGrid2d):
    goals = WavefrontFrontierDetector.getGoals(pose, map)
    if len(goals) == 0:
      return None

    goal = None
    largestDist = 0
    for f in goals:
      dist = math.sqrt(((f[0] - pose.position.x)**2) + ((f[1] - pose.position.y)**2))
      if  dist > largestDist:
        largestDist = dist
        goal = [f]
    return goal

  @staticmethod
  def getFrontier(pose, map: OccupancyGrid2d):
    fCache = FrontierCache()

    fCache.clear()

    mx, my = map.worldToMap(pose.position.x, pose.position.y)

    freePoint = WavefrontFrontierDetector.findFree(mx, my, map)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = WavefrontFrontierDetector.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
      p = mapPointQueue.pop(0)

      if p.classification & WavefrontFrontierDetector.MapClosed.value != 0:
        continue

      if WavefrontFrontierDetector.isFrontierPoint(p, map, fCache):
        p.classification = p.classification | WavefrontFrontierDetector.FrontierOpen.value
        frontierQueue = [p]
        newFrontier = []

        while len(frontierQueue) > 0:
          q = frontierQueue.pop(0)

          if q.classification & (WavefrontFrontierDetector.MapClosed.value | WavefrontFrontierDetector.FrontierClosed.value) != 0:
            continue

          if WavefrontFrontierDetector.isFrontierPoint(q, map, fCache):
            newFrontier.append(q)
            for w in WavefrontFrontierDetector.getNeighbors(q, map, fCache):
              if w.classification & (WavefrontFrontierDetector.FrontierOpen.value | WavefrontFrontierDetector.FrontierClosed.value | WavefrontFrontierDetector.MapClosed.value) == 0:
                w.classification = w.classification | WavefrontFrontierDetector.FrontierOpen.value
                frontierQueue.append(w)

          q.classification = q.classification | WavefrontFrontierDetector.FrontierClosed.value
        
        newFrontierCords = []
        for x in newFrontier:
          x.classification = x.classification | WavefrontFrontierDetector.MapClosed.value
          newFrontierCords.append(map.mapToWorld(x.mapX, x.mapY))

        if len(newFrontier) > WavefrontFrontierDetector.MIN_FRONTIER_SIZE:
          frontiers.append(WavefrontFrontierDetector.centroid(newFrontierCords))

      for v in WavefrontFrontierDetector.getNeighbors(p, map, fCache):
        if v.classification & (WavefrontFrontierDetector.MapOpen.value | WavefrontFrontierDetector.MapClosed.value) == 0:
          if any(map.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, map, fCache)):
            v.classification = v.classification | WavefrontFrontierDetector.MapOpen.value
            mapPointQueue.append(v)

      p.classification = p.classification | WavefrontFrontierDetector.MapClosed.value

    return frontiers

  @staticmethod
  def isFrontierPoint(point, map: OccupancyGrid2d, fCache):
    if map.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
      return False

    hasFree = False
    for n in WavefrontFrontierDetector.getNeighbors(point, map, fCache):
      cost = map.getCost(n.mapX, n.mapY)

      if cost > WavefrontFrontierDetector.OCC_THRESHOLD:
        return False

      if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
        hasFree = True

    return hasFree

  @staticmethod
  def findFree(mx, my, map: OccupancyGrid2d):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
      loc = bfs.pop(0)

      if map.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
        return (loc.mapX, loc.mapY)

      for n in WavefrontFrontierDetector.getNeighbors(loc, map, fCache):
        if n.classification & WavefrontFrontierDetector.MapClosed.value == 0:
          n.classification = n.classification | WavefrontFrontierDetector.MapClosed.value
          bfs.append(n)

    return (mx, my)

  @staticmethod
  def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length

  @staticmethod
  def getNeighbors(point, map: OccupancyGrid2d, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
      for y in range(point.mapY - 1, point.mapY + 2):
        if (x > 0 and x < map.getSizeX() and y > 0 and y < map.getSizeY()):
          neighbors.append(fCache.getPoint(x, y))

    return neighbors
