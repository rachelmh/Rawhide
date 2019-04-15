#!/usr/bin/python

#RRT* RMH 3_21 nD
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import random
import numpy as np
import copy
from scipy.spatial import distance
import time
import pdb


class Node:
    def __init__(self, q,parent_id=None,cost=0):
      self.q=q
      self.parent=parent_id
      self.cost=cost


class RRTStarNd:
    def __init__(self, numiters,expandDis, jointLimits,start_pos,end_pos, NearNodeSearchRadius,goalSampleRate,goalTol,drawGraph=0):
        self.numiters=numiters
        self.start_node=Node(start_pos)
        self.end_node=Node(end_pos)
        self.jointLimits=jointLimits
        self.nodelist=[]
        #self.nodelist.append(self.start_node)
        self.NearNodeSearchRadius=NearNodeSearchRadius
        self.expandDis=expandDis
        self.goalSampleRate=goalSampleRate
        self.goalTol=goalTol
        self.node_q_list=[]
        self.drawgraph=drawGraph







    @profile
    def get_random_node(self):
        if random.randint(0, 100) > self.goalSampleRate:
            #this is where I put my joint limits
            rnd_q=[]
            for i in range(len(self.start_node.q)):
                    rnd_q.append(random.uniform(self.jointLimits[0][i],self.jointLimits[1][i]))
            rnd_node=Node(rnd_q)
        else:  # goal point sampling
            rnd_node=self.end_node


        return rnd_node

    @profile
    def closest_node_index(self,node, nodes):
        closest_index = distance.cdist([node], nodes).argmin()
        #print(closest_index,'closest_index')
        return nodes[closest_index]

    @profile
    def get_nearest_node_index(self,rand_node):


        #print('nodelist',nodeList)
        nodeqlist=[node.q for node in self.nodelist]
        #min_index=self.closest_node_index(rand_node.q,nodeqlist)
        dlist=self.dist_list(node.q,nodeqlist)

        #min_index=dlist.index(min(dlist))
        min_index=np.argmin(dlist)

        return min_index

    @profile
    def dist_list(self,nodeq, nodesq):
        distancelist = distance.cdist([nodeq], nodesq)
        return distancelist[0]


    @profile
    def steer(self,nearest_node_idx,rand_node):
        #print(nearest_node_idx)
        #nearestNode=self.nodelist(nearest_node_idx)
        #print('nodelist',self.nodelist,nearest_node_idx)
        nearestNode=self.nodelist[nearest_node_idx]
        new_node=rand_node

        nearest_node_q=np.array(nearestNode.q)
        new_node_q=np.array(new_node.q)
        diff=new_node_q-nearest_node_q
        magnitude=np.linalg.norm(diff)
        if magnitude <= self.expandDis:
            pass
        else:
            new_node_q=new_node_q+ self.expandDis*(diff/magnitude)
            new_node.q=new_node_q

        new_node.cost=float('inf')
        new_node.parent=None
        return new_node

    @profile
    def obstacle_free(self):
        return True

    @profile
    def collision_free(self):
        return True

    @profile
    def find_closest_collision_free_pt(self,idx_q_dist_sorted_by_dist):
        if len(idx_q_dist_sorted_by_dist) ==0:
            return 0
        for point in idx_q_dist_sorted_by_dist:

            if self.collision_free():
                return point
        return 0

    @profile
    def near_verticies_index_list(self, new_node):

        r=self.NearNodeSearchRadius
        #nodeqlist=[node.q for node in self.nodelist]

        dlist=self.dist_list(new_node.q,self.node_q_list)

        nnode=len(self.nodelist)
        #r=50.0 * math.sqrt((math.log(nnode) / nnode))
        #r=np.linalg.norm(np.array(dlist))/2
        #r=10
        r=self.NearNodeSearchRadius
        #print(dlist)
        #near_idxs=[dist_list[0].index(i) for i in dist_list if i <= r ** 2]
        #r=.2
        near_idxs=[i for i,v in enumerate(dlist <= r) if v]

        return near_idxs

    @profile
    def choose_parent(self,new_node,near_verticies_index_list):
        if len(near_verticies_index_list) ==0:
            return new_node

        #near_verticies_index_list is the list of indexes in self.nodelist for the near verticies

        nearNodes=[]
        near_nodes_q_list=[]
        for n in near_verticies_index_list:
            nearNodes.append(self.nodelist[n])
            near_nodes_q_list.append(self.node_q_list[n])  #gather the near nodes together


        #near_node_q_list=[node.q for node in nearNodes]
        near_node_dist_list=self.dist_list(new_node.q,near_nodes_q_list)
        #print('NEARNODES',len(nearNodes),near_node_dist_list)

        cost_list=[]
        for n in range(len(nearNodes)):
            cost_list.append(nearNodes[n].cost+near_node_dist_list[n])
        #print('cost_list',cost_list)
        min_cost=min(cost_list)
        minind=near_verticies_index_list[cost_list.index(min_cost)]
        #pdb.set_trace()
        if min_cost==float('inf'):
            return new_node
        new_node.cost=min_cost
        new_node.parent=minind
        return new_node




        #idx_q_dist=zip(near_verticies_index_list,near_node_q_list,near_node_dist_list)
        #print('idx_q_dist',idx_q_dist)
        # idx=np.argmin(near_node_dist_list)
        # pdb.set_trace()
        #
        # #point=self.find_closest_collision_free_pt(idx_q_dist)
        # new_node.cost=nearNodes[idx].cost+near_node_dist_list[idx]
        # new_node.parent=idx ####

        # if point==0:
        #     return new_node
        # else:
        #     new_node.cost=self.nodelist[point[0]].cost +point[2]
        #     new_node.parent=point[0]
        #     return new_node

    @profile
    def rewire(self, new_node,near_verticies_index_list,):
        nnode=len(self.nodelist)
        nearNodes=[]
        near_node_q_list=[]
        for n in near_verticies_index_list:
            nearNodes.append(self.nodelist[n])
            near_node_q_list.append(self.node_q_list[n])


        #near_node_q_list=[node.q for node in nearNodes]
        new_node_q=np.array(new_node.q)
        for node in nearNodes:
            near_node_q=np.array(node.q)
            diff=new_node_q-near_node_q
            d=np.linalg.norm(diff)

            scost= new_node.cost + d
            if node.cost > scost:
                if self.collision_free():
                    print(node.parent,node.cost)
                    #pdb.set_trace()
                    node.parent = nnode -1
                    node.cost=scost


    @profile
    def get_best_last_index(self):
        dist_to_goal_list=[self.calc_dist_to_goal(node) for node in self.nodelist]
        goal_idxs = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.goalTol]
        #pdb.set_trace()
        if len(goal_idxs) == 0:
            return None

        mincost = min([self.nodelist[i].cost for i in goal_idxs])
        #pdb.set_trace()
        for i in goal_idxs:
            if self.nodelist[i].cost == mincost:
                return i
        return None

    @profile
    def calc_dist_to_goal(self, node):
        node_q=np.array(node.q)
        end_q=self.end_node.q
        diff=node_q-end_q
        d=np.linalg.norm(diff)
        return d

    @profile
    def gen_final_course(self, goal_idx):
        #path = [[self.end.x, self.end.y,self.end.z]]
        path=[self.end_node]

        while self.nodelist[goal_idx].parent is not None:
            #pdb.set_trace()
            node = self.nodelist[goal_idx]
            #path.append([node.x, node.y, node.z])
            path.append(node)
            goal_idx = node.parent
        #path.append([self.start.x, self.start.y,self.start.z])
        path.append(self.start_node)
        return path

    @profile
    def DrawGraph(self,random_node=None):
        plt.clf()
        jspacing=[1,2,3,4,5,6,7,8,9,10,11,12,13,14]
        # plt.plot(self.start_node.q,jspacing,'bo')
        # plt.plot(self.end_node.q,jspacing,'rx')
        plt.plot(self.start_node.q[0],self.start_node.q[1],'bo',linewidth=2, markersize=12)
        plt.plot(self.end_node.q[0],self.end_node.q[1],'rx',linewidth=2, markersize=12)
        if random_node is not None:
            random_node_q=random_node.q.tolist()
            #plt.plot(random_node_q,jspacing, "^k")
            plt.plot(random_node_q[0],random_node_q[1], "^k")
        #print('rnq',random_node_q)
        node_q_list=[node.q for node in self.nodelist if node.parent is not None]
        node_q_list=[node.q for node in self.nodelist]# if node.parent is not None]

        #for nodeq in node_q_list:
            #plt.plot(nodeq,[1,2,3,4,5,6,7,8,9,10,11,12,13,14],'g+')
        for node in self.nodelist:
            plt.plot(node.q[0],node.q[1],'g+')
            if node.parent is not None:
                plt.plot([node.q[0], self.nodelist[node.parent].q[0]],[node.q[1], self.nodelist[node.parent].q[1]] , "g-")

        plt.axis([0, 15, 0, 16])
        plt.grid(True)
        plt.pause(0.005)


    @profile
    def main_alg(self):
        self.nodelist=[self.start_node]
        self.node_q_list=[self.start_node.q]
        for i in range(self.numiters):
            rand_node=self.get_random_node()
            nearest_node_idx=self.get_nearest_node_index(rand_node)
            print('nnidx',nearest_node_idx)
            new_node=self.steer(nearest_node_idx,rand_node)
            if self.obstacle_free:
                near_node_idx_list=self.near_verticies_index_list(new_node)
                new_node=self.choose_parent(new_node,near_node_idx_list)
                #print(new_node.parent)
                self.nodelist.append(new_node)
                self.node_q_list.append(new_node.q)
                #print('nodes',len(self.nodelist))
                #time.sleep(0.5)
                self.rewire(new_node, near_node_idx_list)
                #self.DrawGraph(rand_node)
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

def main():
    print("Start rrt planning")
    ni=1000
    ed=.1
    #jl=[ [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0] ]
    #sp=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    #ep=[5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0]
    jl=[[0.0,0.0],[15.0,15.0]]
    sp=[0.0,0.0]
    ep=[5.0,5.0]
    nnsr=3
    gsr=.3
    gt=.3
    drawgraph=0
    rrts=RRTStarNd( numiters=ni,expandDis=ed, jointLimits=jl,start_pos=sp,end_pos=ep, NearNodeSearchRadius=nnsr,goalSampleRate=gsr,goalTol=gt, drawGraph=drawgraph)
    path = rrts.main_alg()


    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        path_q_list=[node.q for node in path]
        print(path_q_list)
        rrts.DrawGraph()
        x=[item[0] for item in path_q_list]
        y=[item[1] for item in path_q_list]
        plt.plot(x,y,'r-')

        plt.pause(0.01)

        plt.show()


if __name__ == '__main__':
    main()
