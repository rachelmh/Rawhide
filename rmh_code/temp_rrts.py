#RRT* RMH 3_20_19

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

show_animation = True

class Node():
    """
    RRT Node
    """

    def __init__(self, jr0, jr1,jr2,jr3,jr4,jr5,jr6,jl0,jl1,jl2,jl3,jl4,jl5,jl6):
        self.jr0 = jr0
        self.jr1 = jr1
        self.jr2 = jr2
        self.jr3 = jr3
        self.jr4 = jr4
        self.jr5 = jr5
        self.jr6 = jr6
        self.jl0 = jl0
        self.jl1 = jl1
        self.jl2 = jl2
        self.jl3 = jl3
        self.jl4 = jl4
        self.jl5 = jl5
        self.jl6 = jl6
        self.cost = 0.0
        self.parent = None



class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, jointLimits,
                 expandDis=.5, goalSampleRate=30, maxIter=700):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1],start[2],start[3],start[4],start[5],start[6],start[7],start[8],start[9],start[10],start[11],start[12],start[13])
        self.end = Node(goal[0], goal[1],goal[2],goal[3],goal[4],goal[5],goal[6],goal[7],goal[8],goal[9],goal[10],goal[11],goal[12],goal[13])
        #self.minrand = randArea[0]
        #self.maxrand = randArea[1]

        self.minJLjr0 = jointLimits[0][0]
        self.maxJLjr0 = jointLimits[1][0]
        self.minJLjr1 = jointLimits[0][1]
        self.maxJLjr1 = jointLimits[1][1]
        self.minJLjr2 = jointLimits[0][2]
        self.maxJLjr2 = jointLimits[1][2]
        self.minJLjr3 = jointLimits[0][3]
        self.maxJLjr3 = jointLimits[1][3]
        self.minJLjr4 = jointLimits[0][4]
        self.maxJLjr4 = jointLimits[1][4]
        self.minJLjr5 = jointLimits[0][5]
        self.maxJLjr5 = jointLimits[1][5]
        self.minJLjr6 = jointLimits[0][6]
        self.maxJLjr6 = jointLimits[1][6]
        self.minJLjl0 = jointLimits[0][7]
        self.maxJLjl0 = jointLimits[1][7]
        self.minJLjl1 = jointLimits[0][8]
        self.maxJLjl1 = jointLimits[1][8]
        self.minJLjl2 = jointLimits[0][9]
        self.maxJLjl2 = jointLimits[1][9]
        self.minJLjl3 = jointLimits[0][10]
        self.maxJLjl3 = jointLimits[1][10]
        self.minJLjl4 = jointLimits[0][11]
        self.maxJLjl4 = jointLimits[1][11]
        self.minJLjl5 = jointLimits[0][12]
        self.maxJLjl5 = jointLimits[1][12]
        self.minJLjl6 = jointLimits[0][13]
        self.maxJLjl6 = jointLimits[1][13]


        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):

            rnd = self.get_random_point()
            #print('rand',rnd)
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.__CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

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
            #dx = newNode.x - self.nodeList[i].x
            #dy = newNode.y - self.nodeList[i].y
            #dz = newNode.z - self.nodeList[i].z
            djr0 = newNode.jr0 - self.nodeList[i].jr0
            djr1 = newNode.jr1 - self.nodeList[i].jr1
            djr2 = newNode.jr2- self.nodeList[i].jr2
            djr3 = newNode.jr3- self.nodeList[i].jr3
            djr4 = newNode.jr4- self.nodeList[i].jr4
            djr5 = newNode.jr5- self.nodeList[i].jr5
            djr6 = newNode.jr6- self.nodeList[i].jr6
            djl0 = newNode.jl0- self.nodeList[i].jl0
            djl1 = newNode.jl1- self.nodeList[i].jl1
            djl2 = newNode.jl2- self.nodeList[i].jl2
            djl3 = newNode.jl3- self.nodeList[i].jl3
            djl4 = newNode.jl4- self.nodeList[i].jl4
            djl5 = newNode.jl5- self.nodeList[i].jl5
            djl6 = newNode.jl6- self.nodeList[i].jl6

            #d = math.sqrt(dx ** 2 + dy ** 2+ dz **2 )

            d = math.sqrt(djr0 ** 2 + djr1 ** 2+ djr2 **2 + djr3 ** 2+ djr4 **2+ djr5 ** 2+ djr6 **2+ djl0 ** 2+ djl1 **2+ djl2 ** 2+ djl3 **2+ djl4 ** 2+ djl5 **2+ djl6 ** 2)
            #theta = math.atan2(dy, dx)
            if d==0:
                mag=np.zeros(4)
            else:
                mag=np.array([ djr0,djr1 , djr2 , djr3 , djr4 ,djr5 , djr6 ,djl0, djl1 ,djl2, djl3 , djl4 ,djl5 , djl6 ])/d

            #print(mag,'choose_parent',dy , dx  ,d)
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
        #theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(rnd[0], rnd[1], rnd[2],rnd[3],rnd[4],rnd[5],rnd[6],rnd[7],rnd[8],rnd[9],rnd[10],rnd[11],rnd[12],rnd[13])

        currentDistance = math.sqrt((rnd[0] - nearestNode.jr0) ** 2 + (rnd[1] - nearestNode.jr1) ** 2+ (rnd[2] - nearestNode.jr2) ** 2+ (rnd[3] - nearestNode.jr3) ** 2+ (rnd[4] - nearestNode.jr4) ** 2+ (rnd[5] - nearestNode.jr5) ** 2+ (rnd[6] - nearestNode.jr6) ** 2+ (rnd[7] - nearestNode.jl0) ** 2+ (rnd[8] - nearestNode.jl1) ** 2+ (rnd[9] - nearestNode.jl2) ** 2+ (rnd[10] - nearestNode.jl3) ** 2+ (rnd[11] - nearestNode.jl4) ** 2+ (rnd[12] - nearestNode.jl5) ** 2+ (rnd[13] - nearestNode.jl6) ** 2)


        if currentDistance== 0:
            mag=np.zeros(14)
        else:
            mag=np.array([ (rnd[0] - nearestNode.jr0),(rnd[1] - nearestNode.jr1) , (rnd[2] - nearestNode.jr2) , (rnd[3] - nearestNode.jr3) ,(rnd[4] - nearestNode.jr4) ,(rnd[5] - nearestNode.jr5) , (rnd[6] - nearestNode.jr6), (rnd[7] - nearestNode.jl0),(rnd[8] - nearestNode.jl1) , (rnd[9] - nearestNode.jl2) , (rnd[10] - nearestNode.jl3) , (rnd[11] - nearestNode.jl4) , (rnd[12] - nearestNode.jl5), (rnd[13] - nearestNode.jl6) ])/currentDistance
        #print(mag,'steer',(rnd[1] - nearestNode.y) , (rnd[0] - nearestNode.x) ,currentDistance)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            #newNode.x = nearestNode.x + self.expandDis * math.cos(theta)
            #newNode.y = nearestNode.y + self.expandDis * math.sin(theta)
            #newNode.z = nearestNode.z + self.expandDis * mag[2]
            #newNode.x = nearestNode.x + self.expandDis * mag[1]
            #newNode.y = nearestNode.y + self.expandDis * mag[0]
            newNode.jr0 = nearestNode.jr0 +self.expandDis * mag[0]
            newNode.jr1 = nearestNode.jr1 +self.expandDis * mag[1]
            newNode.jr2 = nearestNode.jr2+self.expandDis * mag[2]
            newNode.jr3 = nearestNode.jr3+self.expandDis * mag[3]
            newNode.jr4 = nearestNode.jr4+self.expandDis * mag[4]
            newNode.jr5 = nearestNode.jr5+self.expandDis * mag[5]
            newNode.jr6 = nearestNode.jr6+self.expandDis * mag[6]
            newNode.jl0 = nearestNode.jl0+self.expandDis * mag[7]
            newNode.jl1 = nearestNode.jl1+self.expandDis * mag[8]
            newNode.jl2 = nearestNode.jl2+self.expandDis * mag[9]
            newNode.jl3 = nearestNode.jl3+self.expandDis * mag[10]
            newNode.jl4 = nearestNode.jl4+self.expandDis * mag[11]
            newNode.jl5 = nearestNode.jl5+self.expandDis * mag[12]
            newNode.jl6 = nearestNode.jl6+self.expandDis * mag[13]
            #print(self.expandDis * mag[1],self.expandDis * mag[0], self.expandDis * math.cos(theta), self.expandDis * math.cos(theta))
        newNode.cost = float("inf")
        newNode.parent = None
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            #this is where I put my joint limits
            # rnd = [random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand),
            #        random.uniform(self.minrand, self.maxrand)]

            rnd=[random.uniform(self.minJLjr0,  self.maxJLjr0),
                random.uniform(self.minJLjr1, self.maxJLjr1),
                random.uniform(self.minJLjr2 ,self.maxJLjr2) ,
                random.uniform(self.minJLjr3 ,self.maxJLjr3),
                random.uniform(self.minJLjr4 ,self.maxJLjr4 ),
                random.uniform(self.minJLjr5 ,self.maxJLjr5 ),
                random.uniform(self.minJLjr6 ,self.maxJLjr6),
                random.uniform(self.minJLjl0 ,self.maxJLjl0),
                random.uniform(self.minJLjl1 ,self.maxJLjl1 ),
                random.uniform(self.minJLjl2 ,self.maxJLjl2 ),
                random.uniform(self.minJLjl3 ,self.maxJLjl3 ),
                random.uniform(self.minJLjl4 ,self.maxJLjl4 ),
                random.uniform(self.minJLjl5 ,self.maxJLjl5),
                random.uniform(self.minJLjl6 , self.maxJLjl6) ]
        else:  # goal point sampling
            #rnd = [self.end.x, self.end.y, self.end.z]
            rnd=[self.end.jr0, self.end.jr1,self.end.jr2,self.end.jr3,self.end.jr4,self.end.jr5,self.end.jr6,self.end.jl0,self.end.jl1,self.end.jl2,self.end.jl3,self.end.jl4,self.end.jl5,self.end.jl6]

        return rnd

    def get_best_last_index(self):

        # disglist = [self.calc_dist_to_goal(
        #     node.x, node.y,node.z) for node in self.nodeList]

        disglist = [self.calc_dist_to_goal(node.jr0, node.jr1,node.jr2,node.jr3,node.jr4,node.jr5,node.jr6,node.jl0,node.jl1,node.jl2,node.jl3,node.jl4,node.jl5,node.jl6) for node in self.nodeList]

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
        #path = [[self.end.x, self.end.y,self.end.z]]
        path=[[self.end.jr0, self.end.jr1,self.end.jr2,self.end.jr3,self.end.jr4,self.end.jr5,self.end.jr6,self.end.jl0,self.end.jl1,self.end.jl2,self.end.jl3,self.end.jl4,self.end.jl5,self.end.jl6]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            #path.append([node.x, node.y, node.z])
            path.append([node.jr0, node.jr1,node.jr2,node.jr3,node.jr4,node.jr5,node.jr6,node.jl0,node.jl1,node.jl2,node.jl3,node.jl4,node.jl5,node.jl6])
            goalind = node.parent
        #path.append([self.start.x, self.start.y,self.start.z])
        path.append([self.start.jr0, self.start.jr1,self.start.jr2,self.start.jr3,self.start.jr4,self.start.jr5,self.start.jr6,self.start.jl0,self.start.jl1,self.start.jl2,self.start.jl3,self.start.jl4,self.start.jl5,self.start.jl6])
        return path

    #def calc_dist_to_goal(self, x, y,z):
    def calc_dist_to_goal(self, jr0, jr1,jr2,jr3,jr4,jr5,jr6,jl0,jl1,jl2,jl3,jl4,jl5,jl6):
        a= np.linalg.norm([jr0-self.end.jr0, jr1-self.end.jr1, jr2-self.end.jr2, jr3-self.end.jr3, jr4-self.end.jr4, jr5-self.end.jr5, jr6-self.end.jr6, jl0-self.end.jl0, jl1-self.end.jl1, jl2-self.end.jl2, jl3- self.end.jl3, jl4-self.end.jl4,jl5-self.end.jl5, jl6-self.end.jl6])
        #print('a',a)
        return a
        #return np.linalg.norm([x - self.end.x, y - self.end.y,z - self.end.z])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0

        dlist=[ (node.jr0 - newNode.jr0)** 2 +(node.jr1 - newNode.jr1) ** 2 + (node.jr2 - newNode.jr2) ** 2 + (node.jr3 - newNode.jr3) ** 2 +(node.jr4 - newNode.jr4) ** 2 +(node.jr5 - newNode.jr5) ** 2 + (node.jr6 - newNode.jr6)** 2 + (node.jl0 - newNode.jl0)** 2 + (node.jl1 - newNode.jl1) ** 2 + (node.jl2 - newNode.jl2) ** 2 + (node.jl3 - newNode.jl3) ** 2 + (node.jl4 - newNode.jl4) ** 2 + (node.jl5 - newNode.jl5)** 2 + (node.jl6 - newNode.jl6)**2 for node in self.nodeList]
        #dlist = [(node.x - newNode.x) ** 2 +
        #         (node.y - newNode.y) ** 2 + (node.z - newNode.z) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            #dx = newNode.x - nearNode.x
            #dy = newNode.y - nearNode.y
            #dz = newNode.z - nearNode.z
            #d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            djr0 = newNode.jr0 - nearNode.jr0
            djr1 = newNode.jr1 - nearNode.jr1
            djr2 = newNode.jr2- nearNode.jr2
            djr3 = newNode.jr3- nearNode.jr3
            djr4 = newNode.jr4- nearNode.jr4
            djr5 = newNode.jr5- nearNode.jr5
            djr6 = newNode.jr6- nearNode.jr6
            djl0 = newNode.jl0- nearNode.jl0
            djl1 = newNode.jl1- nearNode.jl1
            djl2 = newNode.jl2- nearNode.jl2
            djl3 = newNode.jl3- nearNode.jl3
            djl4 = newNode.jl4- nearNode.jl4
            djl5 = newNode.jl5- nearNode.jl5
            djl6 = newNode.jl6- nearNode.jl6
            d = math.sqrt(djr0 ** 2 + djr1 ** 2+ djr2 **2 + djr3 ** 2+ djr4 **2+ djr5 ** 2+ djr6 **2+ djl0 ** 2+ djl1 **2+ djl2 ** 2+ djl3 **2+ djl4 ** 2+ djl5 **2+ djl6 ** 2)



            scost = newNode.cost + d

            if nearNode.cost > scost:
                #theta = math.atan2(dy, dx)
                if d==0:
                    mag=np.zeros(14)
                else:
                    #mag=np.array([ (dy) , (dx) ,dz ])/d
                    mag=np.array([ djr0,djr1 , djr2 , djr3 , djr4 ,djr5 , djr6 ,djl0, djl1 ,djl2, djl3 , djl4 ,djl5 , djl6 ])/d
                #print(mag,'rewire',dy,dx,d)
                if self.check_collision_extend(nearNode, mag,d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode,mag, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):

            #tmpNode.x += self.expandDis * math.cos(theta)
            #tmpNode.y += self.expandDis * math.sin(theta)
            #tmpNode.z += self.expandDis * mag[2]
            #tmpNode.x += self.expandDis * mag[1]
            #tmpNode.y += self.expandDis * mag[0]

            tmpNode.jr0 += self.expandDis * mag[0]
            tmpNode.jr1 +=self.expandDis * mag[1]
            tmpNode.jr2 +=self.expandDis * mag[2]
            tmpNode.jr3 += self.expandDis * mag[3]
            tmpNode.jr4 += self.expandDis * mag[4]
            tmpNode.jr5 += self.expandDis * mag[5]
            tmpNode.jr6 += self.expandDis * mag[6]
            tmpNode.jl0 += self.expandDis * mag[7]
            tmpNode.jl1 += self.expandDis * mag[8]
            tmpNode.jl2 += self.expandDis * mag[9]
            tmpNode.jl3 += self.expandDis * mag[10]
            tmpNode.jl4 += self.expandDis * mag[11]
            tmpNode.jl5 += self.expandDis * mag[12]
            tmpNode.jl6 += self.expandDis * mag[13]
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    # def DrawGraph(self, rnd=None):
    #     """
    #     Draw Graph
    #     """
    #     plt.clf()
    #     if rnd is not None:
    #         plt.plot(rnd[0], rnd[1], "^k")
    #     for node in self.nodeList:
    #         if node.parent is not None:
    #             plt.plot([node.x, self.nodeList[node.parent].jr0], [
    #                      node.y, self.nodeList[node.parent].jr1], "g+")

    #     for (ox, oy, oz,size) in self.obstacleList:
    #         plt.plot(ox, oy, "ok", ms=30 * size)

    #     plt.plot(self.start.x, self.start.y, "xr")
    #     plt.plot(self.end.x, self.end.y, "xr")
    #     plt.axis([-2, 15, -2, 15])
    #     plt.grid(True)
    #     plt.pause(0.01)

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        # if rnd is not None:
        #     plt.plot(rnd[0], rnd[1], "^k")
        # for node in self.nodeList:
        #     if node.parent is not None:
        #         plt.plot([node.jr0, self.nodeList[node.parent].jr0], [
        #                  node.jr1, self.nodeList[node.parent].jr1], "g-")

        # for (ox, oy,size) in self.obstacleList:
        #     plt.plot(ox, oy, "ok", ms=30 * size)

        # plt.plot(self.start.jr0, self.start.jr1, "xr")
        # plt.plot(self.end.jr0, self.end.jr1, "xr")

        # if rnd is not None:
        #     plt.plot(rnd[2], rnd[3],"+k")
        # for node in self.nodeList:
        #     if node.parent is not None:
        #         plt.plot([node.jr2, self.nodeList[node.parent].jr2], [
        #                  node.jr3, self.nodeList[node.parent].jr3], "b-")


        # plt.plot(self.start.jr2, self.start.jr3, "xy")
        # plt.plot(self.end.jr2, self.end.jr3, "xy")

        if rnd is not None:
            plt.plot(rnd[0], 0, "^k")
            plt.plot(rnd[1], 0.5, "^k")
            plt.plot(rnd[2], 1.0, "^k")
            plt.plot(rnd[3], 1.50, "^k")
            plt.plot(rnd[4], 2.0, "^k")
            plt.plot(rnd[5], 2.50, "^k")
            plt.plot(rnd[6], 3.0, "^k")
            plt.plot(rnd[7], 3.50, "^k")
            plt.plot(rnd[8], 4.0, "^k")
            plt.plot(rnd[9], 4.50, "^k")
            plt.plot(rnd[10], 5.0, "^k")
            plt.plot(rnd[11], 5.50, "^k")
            plt.plot(rnd[12], 6.0, "^k")
            plt.plot(rnd[13], 6.50, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.jr0, self.nodeList[node.parent].jr0], [0, 0], "g+")
                plt.plot([node.jr1, self.nodeList[node.parent].jr1], [0.5, 0.5], "g+")
                plt.plot([node.jr2, self.nodeList[node.parent].jr2], [1.0, 1.0], "g+")
                plt.plot([node.jr3, self.nodeList[node.parent].jr3], [1.50, 1.50], "g+")
                plt.plot([node.jr4, self.nodeList[node.parent].jr4], [2.0, 2.0], "g+")
                plt.plot([node.jr5, self.nodeList[node.parent].jr5], [2.50, 2.50], "g+")
                plt.plot([node.jr6, self.nodeList[node.parent].jr6], [3.0, 3.0], "g+")
                plt.plot([node.jl0, self.nodeList[node.parent].jl0], [3.50, 3.50], "g+")
                plt.plot([node.jl1, self.nodeList[node.parent].jl1], [4.0, 4.0], "g+")
                plt.plot([node.jl2, self.nodeList[node.parent].jl2], [4.50, 4.50], "g+")
                plt.plot([node.jl3, self.nodeList[node.parent].jl3], [5.0, 5.0], "g+")
                plt.plot([node.jl4, self.nodeList[node.parent].jl4], [5.50, 5.50], "g+")
                plt.plot([node.jl5, self.nodeList[node.parent].jl5], [6.0, 6.0], "g+")
                plt.plot([node.jl6, self.nodeList[node.parent].jl6], [6.50, 6.50], "g+")

        #plt.plot(self.start.jr0, self.start.jr3, "xy")
        #plt.plot(self.end.jr2, self.end.jr3, "xy")


        plt.axis([0, 15, 0, 15])
        plt.grid(True)
        #plt.pause(0.001)

    def GetNearestListIndex(self, nodeList, rnd):
        #dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
        #         ** 2 + (node.z - rnd[2]) ** 2 for node in nodeList]
        dlist=[ (node.jr0 - rnd[0])** 2 +(node.jr1 - rnd[1]) ** 2 + (node.jr2 - rnd[2]) ** 2 + (node.jr3 - rnd[3]) ** 2 +(node.jr4 - rnd[4]) ** 2 +(node.jr5 - rnd[5]) ** 2 + (node.jr6 - rnd[6])** 2 + (node.jl0 - rnd[7])** 2 + (node.jl1 - rnd[8]) ** 2 + (node.jl2 - rnd[9]) ** 2 + (node.jl3 - rnd[10]) ** 2 + (node.jl4 - rnd[11]) ** 2 + (node.jl5 - rnd[12])** 2 + (node.jl6 - rnd[13])**2 for node in self.nodeList]

        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):
        # for (ox, oy, oz,size) in obstacleList:
        #     dx = ox - node.x
        #     dy = oy - node.y
        #     dz = 0#oz - node.z
        #     d = dx * dx + dy * dy + dz*dz
        #     if d <= size ** 2:
        #         return False  # collision

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
    rrt = RRT(start=[0, 0,0,0,0,0,0,0,0,0,0,0,0,0], goal=[5, 10,10,10,10,10,10,10,10,10,10,10,10,10],
              jointLimits=jointLimits, obstacleList=[])
    path = rrt.Planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        print(path)
        # Draw final path
        if show_animation:
            rrt.DrawGraph()
            #plt.plot([x for (x, y,z) in path], [y for (x, y,z) in path], '-r')
            plt.plot([jr0 for (jr0, jr1,jr2,jr3,jr4,jr5,jr6,jl0,jl1,jl2,jl3,jl4,jl5,jl6) in path],[jr1 for (jr0, jr1,jr2,jr3,jr4,jr5,jr6,jl0,jl1,jl2,jl3,jl4,jl5,jl6) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
