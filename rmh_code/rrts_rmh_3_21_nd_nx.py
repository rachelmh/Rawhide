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
import matplotlib.animation as animation


class Node:
    def __init__(self, q,parent_id=None,cost=0):
      self.q=q
      self.parent=parent_id
      self.cost=cost


class RRTStarNd:
    def __init__(self, numiters,expandDis, jointLimits,start_pos,end_pos, NearNodeSearchRadius,goalSampleRate,goalTol,DrawGraph=0):
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
        self.drawgraph=DrawGraph
        self.goalcounter=0


    def get_random_node(self):
        if random.randint(0, 100) > self.goalSampleRate:
            #this is where I put my joint limits
            rnd_q=[]
            for i in range(len(self.start_node.q)):
                    rnd_q.append(random.uniform(self.jointLimits[0][i],self.jointLimits[1][i]))


            rnd_node=Node(rnd_q)
        else:  # goal point sampling
            if random.randint(0, 100) < 90:
                rnd_node=copy.deepcopy(self.end_node)
            else:
                rnd_node=copy.deepcopy(self.start_node)
                for i in range(len(rnd_node.q)):
                    a=random.uniform(-self.expandDis*2,self.expandDis*2)

                    rnd_node.q[i]=rnd_node.q[i]+a



        return rnd_node

    def closest_node_index(self,node, nodes):
        closest_index = distance.cdist([node], nodes).argmin()
        #print(closest_index,'closest_index')
        return nodes[closest_index]

    def get_nearest_node_index(self,rand_node):


        #print('nodelist',self.nodelist)
        #nodeqlist=[node.q for node in self.nodelist]
        #min_index=self.closest_node_index(rand_node.q,nodeqlist)
        dlist=self.dist_list(rand_node.q,self.node_q_list)

        #min_index=dlist.index(min(dlist))
        min_index=np.argmin(dlist)
        #pdb.set_trace()
        return min_index

    def dist_list(self,nodeq, nodesq):
        distancelist = distance.cdist([nodeq], nodesq,'euclidean')
        return distancelist[0]


    def steer(self,nearest_node_idx,rand_node):
        #print(nearest_node_idx)
        #nearestNode=self.nodelist(nearest_node_idx)
        #print('nodelist',self.nodelist,nearest_node_idx)
        nearestNode=self.nodelist[nearest_node_idx]
        new_node=rand_node

        #nearest_node_q=np.array(nearestNode.q)
        nearest_node_q=self.node_q_list[nearest_node_idx]
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

    def obstacle_free(self):
        return True

    def collision_free(self):
        return True


    def find_closest_collision_free_pt(self,idx_q_dist_sorted_by_dist):
        if len(idx_q_dist_sorted_by_dist) ==0:
            return 0
        for point in idx_q_dist_sorted_by_dist:

            if self.collision_free():
                return point
        return 0


    def near_verticies_index_list(self, new_node):
        nnode=len(self.nodelist)
        r=self.NearNodeSearchRadius * 5* math.sqrt((math.log(nnode) / nnode))
        #print('r',r)

        #nodeqlist=[node.q for node in self.nodelist]

        dlist=self.dist_list(new_node.q,self.node_q_list)


        #r=50.0 * math.sqrt((math.log(nnode) / nnode))
        #r=np.linalg.norm(np.array(dlist))/2
        #r=10
        #r=self.NearNodeSearchRadius
        #print(dlist)
        #near_idxs=[dist_list[0].index(i) for i in dist_list if i <= r ** 2]
        #r=.2
        near_idxs=[i for i,v in enumerate(dlist <= r) if v]

        return near_idxs

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
                    #print(node.parent,node.cost)
                    #pdb.set_trace()
                    node.parent = nnode -1
                    node.cost=scost


    def get_best_last_index(self):
        dist_to_goal_list=[self.calc_dist_to_goal(node) for node in self.nodelist]
        #goal_idxs = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.goalTol]
        goal_idxs = []
        for idx in range(len(dist_to_goal_list)):
            if dist_to_goal_list[idx]<=self.goalTol:
                goal_idxs.append(idx)
        #pdb.set_trace()
        if len(goal_idxs) == 0:
            return None

        #pdb.set_trace()
        mincost = min([self.nodelist[i].cost for i in goal_idxs])
        #pdb.set_trace()
        for i in goal_idxs:
            if self.nodelist[i].cost == mincost:
                return i
        return None

    def calc_dist_to_goal(self, node):
        node_q=np.array(node.q)
        end_q=self.end_node.q
        diff=node_q-end_q
        d=np.linalg.norm(diff)
        return d

    def gen_final_course(self, goal_idx):
        #path = [[self.end.x, self.end.y,self.end.z]]
        path=[]
        #path=[self.end_node]

        while self.nodelist[goal_idx].parent is not None:
            #pdb.set_trace()
            node = self.nodelist[goal_idx]
            #ppath.append([node.x, node.y, node.z])
            path.append(node)
            goal_idx = node.parent

        #path.append([self.start.x, self.start.y,self.start.z])

        if len(path) > 0:
            path.insert(0,self.end_node)
            path.append(self.start_node)

        return path

    def DrawGraph(self,random_node=None):
        plt.clf()
        #jspacing=[1,2,3,4,5,6,7,8,9,10,11,12,13,14]
        jspacing=np.linspace(1,len(self.start_node.q),len(self.start_node.q))
        if len(self.start_node.q) != 2:
           plt.plot(self.start_node.q,jspacing,'bo')
           plt.plot(self.end_node.q,jspacing,'rx')

        else:
           plt.plot(self.start_node.q[0],self.start_node.q[1],'bo',linewidth=2, markersize=12)
           plt.plot(self.end_node.q[0],self.end_node.q[1],'rx',linewidth=2, markersize=12)
        if random_node is not None:
            #random_node_q=random_node.q.tolist()
            random_node_q=random_node.q
            if len(self.start_node.q) != 2:
                plt.plot(random_node_q,jspacing, "^k")
            else:
                plt.plot(random_node_q[0],random_node_q[1], "^k")
        #print('rnq',random_node_q)
        #node_q_list=[node.q for node in self.nodelist if node.parent is not None]
        #node_q_list=[node.q for node in self.nodelist]# if node.parent is not None]
        for node in self.nodelist:
            if len(self.start_node.q) != 2:
                plt.plot(node.q,jspacing,'g+')
                #plt.plot([node.q[0], self.nodelist[node.parent].q[0]],[node.q[1], self.nodelist[node.parent].q[1]] , "g-")
            else:
                    #plt.plot(node.q[0],node.q[1],'g+')
                    if node.parent is not None:
                        plt.plot(node.q[0],node.q[1],'g+')
                        plt.plot([node.q[0], self.nodelist[node.parent].q[0]],[node.q[1], self.nodelist[node.parent].q[1]] , "g-")

        plt.axis([0, 15, 0, 16])
        plt.grid(True)
        plt.pause(0.01)


    def main_alg(self):
        self.nodelist.append(self.start_node)

        self.node_q_list.append(np.array(self.start_node.q))

        for i in range(self.numiters):
            #print('i',i)
            rand_node=self.get_random_node()
            nearest_node_idx=self.get_nearest_node_index(rand_node)
            #print('nnidx',nearest_node_idx,'lennodelist',len(self.nodelist))
            new_node=self.steer(nearest_node_idx,rand_node)
            if self.obstacle_free:
                near_node_idx_list=self.near_verticies_index_list(new_node)
                new_node=self.choose_parent(new_node,near_node_idx_list)
                #print(new_node.parent)
                self.nodelist.append(new_node)
                self.node_q_list.append(new_node.q)
                self.rewire(new_node, near_node_idx_list)
                if self.drawgraph:
                    self.DrawGraph(rand_node)

                    # writer.grab_frame()


        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return []
        path = self.gen_final_course(lastIndex)

        return path

def main():
    print("Start rrt planning")

    #jl=[ [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0,15.0] ]
    #sp=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    #ep=[5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0]
    # jl=[[0.0,0.0,0.0],[15.0,15.0,15.0]]
    # sp=[0.0,0.0,0.0]
    # ep=[5.0,5.0,5.0]

    ed=.5
    jl=[[0.0,0.0],[15.0,15.0]]
    sp=[0.0,0.0]
    ep=[5.0,5.0]
    nnsr=1
    gsr=20
    gt=.1
    drawgraph=1
    ni=100

    rrts=RRTStarNd( numiters=ni,expandDis=ed, jointLimits=jl,start_pos=sp,end_pos=ep, NearNodeSearchRadius=nnsr,goalSampleRate=gsr,goalTol=gt,DrawGraph=drawgraph)
    path = rrts.main_alg()

    attempts_to_find_plan=5


    if path is None or len(path)==0:
        print("Cannot find path")
        notpath=1
    else:
        print("found path!!")
        path_q_list=[node.q for node in path]
        print(path_q_list)
        if len(sp) ==2:

            rrts.DrawGraph()
            x=[item[0] for item in path_q_list]
            y=[item[1] for item in path_q_list]
            plt.plot(x,y,'r-')
            plt.ioff()
            plt.pause(0.1)

            plt.show()
        notpath=0
    count=0

    while notpath and count<attempts_to_find_plan:
        print('count',count)
        rrts.NearNodeSearchRadius=rrts.NearNodeSearchRadius+.5
        path = rrts.main_alg()
        if (path is None or len(path)==0) :
            print("Cannot find path")
            notpath=1
        else:
            print("found path!!")
            print('NNSR',rrts.NearNodeSearchRadius)
            path_q_list=[node.q for node in path]
            print('')
            print(path_q_list)
            if len(sp) ==2:
                plt.clf()
                rrts.DrawGraph()
                x=[item[0] for item in path_q_list]
                y=[item[1] for item in path_q_list]
                plt.plot(x,y,'r-')
                plt.show()
                notpath=0
        count=count+1



    # while notpath and count<attempts_to_find_plan:
    #     nnsr=nnsr+.5
    #     print('rrt2')
    #     rrts2=RRTStarNd( numiters=ni,expandDis=ed, jointLimits=jl,start_pos=sp,end_pos=ep, NearNodeSearchRadius=nnsr,goalSampleRate=gsr,goalTol=gt,DrawGraph=drawgraph)
    #     path2=rrts2.main_alg()
    #     time.sleep(5)
    #     print('rrt1')
    #     rrts.NearNodeSearchRadius=rrts.NearNodeSearchRadius+.5
    #     path = rrts.main_alg()
    #     time.sleep(5)
    #     if (path is None or len(path)==0) and (path2 is None or len(path2)==0):
    #         print("Cannot find path")
    #         notpath=1
    #     else:
    #         print("found path!!")
    #         if path is not None or len(path)!=0:
    #             print("path1")
    #             print('NNSR',rrts.NearNodeSearchRadius)
    #             path_q_list=[node.q for node in path]
    #             print('')
    #             print(path_q_list)
    #             if len(sp) ==2:
    #                 plt.clf()
    #                 rrts.DrawGraph()
    #                 x=[item[0] for item in path_q_list]
    #                 y=[item[1] for item in path_q_list]
    #                 plt.plot(x,y,'r-')
    #
    #                 #plt.pause(0.1)
    #                 plt.show()
    #                 print('pause')
    #                 time.sleep(5)
    #         if path2 is not None or len(path2)!=0:
    #             print('NNSR',rrts2.NearNodeSearchRadius)
    #             print("path2")
    #             path_q_list=[node.q for node in path2]
    #             print('')
    #             print(path_q_list)
    #             if len(sp) ==2:
    #                 rrts.DrawGraph()
    #                 x=[item[0] for item in path_q_list]
    #                 y=[item[1] for item in path_q_list]
    #                 plt.plot(x,y,'r-')
    #                 #plt.ioff()
    #                 #plt.pause(0.1)
    #                 plt.show()
    #                 print('pause')
    #                 time.sleep(5)
    #
    #
    #
    #         notpath=0
    #     count=count+1


if __name__ == '__main__':
    main()
