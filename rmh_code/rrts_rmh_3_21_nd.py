#!/usr/bin/python

#RRT* RMH 3_21 nD
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

show_animation = False
class Node():
    """
    RRT Node
    """

    def __init__(self, q,parent_id=None,cost=0):
      self.q=q
      self.parent_id=parent_id
      self.cost=cost

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, jointLimits,
        expandDis=.3, goalSampleRate=30, maxIter=700):
        """
        Setting Parameter
        start:Start Joint Positions
        goal:Goal Joint Positions

        randArea:Joint Lmits [[min],[max]]
        """
        self.jointLimits=jointLimits
        self.start=Node(start)
        self.end=Node(goal)
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter


    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):

            rnd = self.get_random_point() #generate random configuration

            nind = self.GetNearestListIndex(self.nodeList, rnd)  #get nearest node from tree

            newNode = self.steer(rnd, nind) #steer toward the random new point
            #  print(newNode.cost)

            if self.__CollisionCheck(newNode): #temp, should always return true
                nearinds = self.find_near_nodes(newNode) #find nearest nodes to new node
                newNode = self.choose_parent(newNode, nearinds) #choose a parent for the new node
                self.nodeList.append(newNode)  #append newnode to nodelist
                self.rewire(newNode, nearinds)  #rewire the tree

            if animation and i % 5 == 0:
                self.DrawGraph(rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path


    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:

            dj=[]
            for n in range(len(newNode.q)):
                dj.append(newNode.q[n] - self.nodeList[i].q[n])

            sum=0
            for j in dj:
                sum += j**2

            d=math.sqrt(sum)
            if d==0:
                mag=np.zeros(len(dj))
            else:
                mag=np.array(dj)/d
            if self.check_collision_extend(self.nodeList[i], mag, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
        newNode = Node(rnd)

        dj=[]
        for n in range(len(nearestNode.q)):
            dj.append((rnd[n] - nearestNode.q[n]))

        sum=0
        for j in dj:
            sum += j**2
        currentDistance=math.sqrt(sum)


        if currentDistance== 0:
            mag=np.zeros(len(newNode.q))
        else:
            mag=np.array(dj)/ currentDistance

        if currentDistance <= self.expandDis:
            pass
        else:
            for i in range(len(nearestNode.q)):
                print('myprint',nearestNode.q,mag)
                newNode.q[i]=(nearestNode.q[i] +self.expandDis * mag[i])


        newNode.cost = float("inf")
        newNode.parent = None
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            #this is where I put my joint limits
            rnd=[]
            for i in range(len(self.start.q)):
                    rnd.append(random.uniform(self.jointLimits[0][i],self.jointLimits[1][i]))

        else:  # goal point sampling
            rnd=self.end

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(node.q) for node in self.nodeList]

        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):

        path=[[self.end]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.q])
            goalind = node.parent

        path.append([self.start])
        return path

    def calc_dist_to_goal(self, q):
        dj=[]
        for n in range(len(q)):
            dj.append((q[n]-self.end[n] ))
        a= np.linalg.norm(dj)
        return a

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0

        dlist=[ (node.q[0] - newNode.q[0])** 2 +(node.q[1] - newNode.q[1]) ** 2 + (node.q[2] - newNode.q[2]) ** 2 + (node.q[3] - newNode.q[3]) ** 2 +(node.q[4] - newNode.q[4]) ** 2 +(node.q[5] - newNode.q[5]) ** 2 + (node.q[6] - newNode.q[6])** 2 + (node.q[7] - newNode.q[7])** 2 + (node.q[8] - newNode.q[8]) ** 2 + (node.q[9] - newNode.q[9]) ** 2 + (node.q[10] - newNode.q[10]) ** 2 + (node.q[11] - newNode.q[11]) ** 2 + (node.q[12] - newNode.q[12])** 2 + (node.q[13] - newNode.q[13])**2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds


    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dj=[]
            for n in range(len(newNode.q)):
                dj.append((newNode.q[n]-nearNode.q[n] ))

            for j in dj:
                sum += j**2
            d=math.sqrt(sum)


            scost = newNode.cost + d

            if nearNode.cost > scost:

                if d==0:
                    mag=np.zeros(len(newNode.q))
                else:
                    mag=np.array([dj])/d

                if self.check_collision_extend(nearNode, mag,d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode,mag, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):


            for n in range(len(tempNode.q)):
                tmpNode.q[0] += self.expandDis * mag[n]


            if not self.__CollisionCheck(tmpNode):
                return False

        return True

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()

        if rnd is not None:
            plt.plot([rnd],[0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5], "^k")

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.q, self.nodeList[node.parent].q], [0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5], "g+")





        plt.axis([9, 11, 0, 16])
        plt.grid(True)
        #plt.pause(0.001)

    def GetNearestListIndex(self, nodeList, rnd):
        print('nodelist',nodeList)
        dlist=[ (node.q[0] - rnd[0])** 2 +(node.q[1] - rnd[1]) ** 2 + (node.q[2] - rnd[2]) ** 2 + (node.q[3] - rnd[3]) ** 2 +(node.q[4] - rnd[4]) ** 2 +(node.q[5] -rnd[5]) ** 2 + (node.q[6] - rnd[6])** 2 + (node.q[7] -rnd[7])** 2 + (node.q[8] -rnd[8]) ** 2 + (node.q[9] - rnd[9]) ** 2 + (node.q[10] - rnd[10]) ** 2 + (node.q[11] - rnd[11]) ** 2 + (node.q[12] -rnd[12])** 2 + (node.q[13] - rnd[13])**2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node):

        return True  # safe

def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
   # rrt = RRT(start=[0, 0,0], goal=[10, 10,0],
   #           randArea=[-2, 15], obstacleList=obstacleList)

    jointLimits=[ [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0] ]
    # rrt = RRT(start=[0, 0,0], goal=[10, 10,0],
    #           randArea=[-2, 15], obstacleList=[])
    rrt = RRT(start=[0, 0,0,0,0,0,0,0,0,0,0,0,0,0], goal=[10, 10,10,10,10,10,10,10,10,10,10,10,10,10],
              jointLimits=jointLimits)
    path = rrt.Planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        #print(path)
        # Draw final path
        if show_animation:
            rrt.DrawGraph()
            #plt.plot([x for (x, y,z) in path], [y for (x, y,z) in path], '-r')
            plt.plot([jr0 for (jr0, jr1,jr2,jr3,jr4,jr5,jr6,jl0,jl1,jl2,jl3,jl4,jl5,jl6) in path],[jr1 for (jr0, jr1,jr2,jr3,jr4,jr5,jr6,jl0,jl1,jl2,jl3,jl4,jl5,jl6) in path], '-r')
            plt.grid(True)
            #plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
