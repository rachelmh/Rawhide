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
import pybullet

class pybullet_env:
    def __init__(self,show_pb,numsawyers):

        if show_pb==1:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

        pybullet.resetSimulation()
        import pybullet_data
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = pybullet.loadURDF("plane.urdf",basePosition=[0,0,-1])
        pybullet.setTimeStep(0.0001)
        self.cabinet=pybullet.loadURDF('/home/rachel/rawhide/rmh_code/cylinder_wire.urdf',basePosition=[.7,0,0],useFixedBase=1)
        if numsawyers==2:
            self.sawyer1=pybullet.loadURDF('/home/rachel/rawhide/rmh_code/pybullet_robots-master/data/sawyer_robot/sawyer_description/urdf/sawyer_rmh.urdf',basePosition=[0,-.35,0],useFixedBase=1)
            self.sawyer2=pybullet.loadURDF('/home/rachel/rawhide/rmh_code/pybullet_robots-master/data/sawyer_robot/sawyer_description/urdf/sawyer_rmh.urdf',basePosition=[0,.35,0],useFixedBase=1)
            self.sawyer_joint_indexes=[3,8,9,10,11,13,16]
            p1pos=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
            self.zeropos=[0,0,0,0,0,0,0]
            self.neutralpos=[0.00, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
            for j in range(len(p1pos)):
                pybullet.resetJointState(self.sawyer1,self.sawyer_joint_indexes[j],p1pos[j])
                pybullet.resetJointState(self.sawyer2,self.sawyer_joint_indexes[j],p1pos[j])
            time.sleep(5)
            for j in range(len(self.zeropos)):
                pybullet.resetJointState(self.sawyer1,self.sawyer_joint_indexes[j],self.zeropos[j])
                pybullet.resetJointState(self.sawyer2,self.sawyer_joint_indexes[j],self.zeropos[j])


        else:
            self.sawyer1=pybullet.loadURDF('/home/rachel/rawhide/rmh_code/pybullet_robots-master/data/sawyer_robot/sawyer_description/urdf/sawyer_rmh.urdf',basePosition=[0,-.35,0],useFixedBase=1)
            self.sawyer_joint_indexes=[3,8,9,10,11,13,16]
            p1pos=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
            self.zeropos=[0,0,0,0,0,0,0]
            self.neutralpos=[0.00, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
            for j in range(len(p1pos)):
                pybullet.resetJointState(self.sawyer1,self.sawyer_joint_indexes[j],p1pos[j])

            time.sleep(5)
            for j in range(len(self.zeropos)):
                pybullet.resetJointState(self.sawyer1,self.sawyer_joint_indexes[j],self.zeropos[j])

        #self.cabinet=pybullet.loadURDF('/home/rachel/rawhide/rmh_code/cylinder_wire.urdf',basePosition=[.7,0,0],useFixedBase=1)
        #cabinet=pybullet.loadURDF('/home/rachel/Desktop/RMH_CODE/cabinet_test.urdf',basePosition=[1,1,0])

    def disconnect(self):
        pybullet.disconnect()

    def loadurdf(self,urdffile,pos):
        pybullet.loadURDF(urdffile,basePosition=pos,useFixedBase=1)


    def move_sawyers_to_joint_pos2(self,saw1_p,saw2_p):
        for j in range(len(self.sawyer_joint_indexes)):
            pybullet.resetJointState(self.sawyer1,self.sawyer_joint_indexes[j],saw1_p[j])
            pybullet.resetJointState(self.sawyer2,self.sawyer_joint_indexes[j],saw2_p[j])
        #pybullet.resetBasePositionAndOrientation(self.cabinet,[.7,0,0],[0,0,0,1])

    def move_sawyers_to_joint_pos1(self,saw1_p):

        for j in range(len(self.sawyer_joint_indexes)):
            pybullet.resetJointState(self.sawyer1,self.sawyer_joint_indexes[j],saw1_p[j])

    def get_sawyer_ee_pos(self,sawyerName):
        if sawyerName=='sawyer1':
            sn=self.sawyer1
        else:
            sn=self.sawyer2
        ee2=pybullet.getLinkState(sn,16)
        return ee2


    def pb_col_free2(self,saw1_p,saw2_p):
        self.move_sawyers_to_joint_pos2(saw1_p,saw2_p)
        pybullet.setJointMotorControlArray(self.sawyer1,self.sawyer_joint_indexes,pybullet.POSITION_CONTROL,targetPositions=saw1_p)
        pybullet.setJointMotorControlArray(self.sawyer2,self.sawyer_joint_indexes,pybullet.POSITION_CONTROL,targetPositions=saw2_p)

        pybullet.stepSimulation()
        cp=pybullet.getContactPoints(self.sawyer1,self.sawyer2)
        #cp2= pybullet.getContactPoints(self.sawyer1,self.cabinet)
        #cp3= pybullet.getContactPoints(self.sawyer2,self.cabinet)
        #if len(cp)==0 and len(cp2)==0 and len(cp3)==0:
        if len(cp)==0 :
            return 0
        else:
            return 1

    def pb_col_free1(self,saw1_p):
        self.move_sawyers_to_joint_pos1(saw1_p)
        pybullet.setJointMotorControlArray(self.sawyer1,self.sawyer_joint_indexes,pybullet.POSITION_CONTROL,targetPositions=saw1_p)

        pybullet.stepSimulation()

        #cp=pybullet.getContactPoints(self.sawyer1,self.sawyer2)
        cp= pybullet.getContactPoints(self.sawyer1,self.cabinet)


        #cp3= pybullet.getContactPoints(self.sawyer2,self.cabinet)
        #if len(cp)==0 and len(cp2)==0 and len(cp3)==0:
        #pdb.set_trace()
        if len(cp)==0 :
            return 1
        else:
            return 0
class Node:
    def __init__(self, q,parent_id=None,cost=0):
      self.q=q
      self.parent=parent_id
      self.cost=cost


class RRTStarNd:
    def __init__(self, numiters,expandDis, jointLimits,start_pos,end_pos, NearNodeSearchRadius,goalSampleRate,goalTol,WeightMat,DrawGraph=0,ShowPybullet=0,numsawyers=0):
        self.numiters=numiters
        self.start_node=Node(start_pos)
        self.end_node=Node(end_pos)
        self.jointLimits=jointLimits
        self.nodelist=[]
        #self.nodelist.append(self.start_node)

        self.k=50
        self.goalSampleRate=goalSampleRate
        self.goalTol=goalTol
        self.node_q_list=[]
        self.drawgraph=DrawGraph
        self.goalcounter=0
        self.weightmat=np.array(WeightMat)
        self.show_pb=ShowPybullet
        self.spherefile='/home/rachel/rawhide/rmh_code/sphere.urdf'
        self.path=[]
        self.vol=np.prod(np.absolute(np.array(self.jointLimits[0])-np.array(self.jointLimits[1])))
        d=len(start_pos)
        self.NearNodeSearchRadius=NearNodeSearchRadius
        self.expandDis=expandDis#.5#(2**d)*(1 + (1/d)) * self.vol\
        self.numsawyers=numsawyers
        self.obstacleList=[[1,1,.25],[-1,1,.25],[1,-1,.25],[-1,-1,.25]]
        if self.numsawyers==1 or self.numsawyers==2:
            self.pbsph=1
        else: self.pbsph=0
        self.pb=None
        #PYBULLET
        if self.numsawyers==2:
            self.pb=pybullet_env(self.show_pb,self.numsawyers)
            ones=[1,1,1,1,1,1,1]
            zeros=[0,0,0,0,0,0,0]
            zeros1=[.35,0,0,0,0,0,0]
            print('starttest')
            saw1_p_s=start_pos[0:7]
            saw2_p_s=start_pos[7:]
            print('collision free START?',self.pb.pb_col_free2(saw1_p_s,saw2_p_s))
            time.sleep(2)
            saw1_p=end_pos[0:7]
            saw2_p=end_pos[7:]
            print('collision free END?',self.pb.pb_col_free2(saw1_p,saw2_p))
            time.sleep(2)

        elif self.numsawyers==1:
            self.pb=pybullet_env(self.show_pb,self.numsawyers)
            ones=[1,1,1,1,1,1,1]
            zeros=[0,0,0,0,0,0,0]
            zeros1=[.35,0,0,0,0,0,0]
            print('starttest')
            saw1_p_s=start_pos
            #saw2_p_s=start_pos[7:]
            print('collision free START?',self.pb.pb_col_free1(saw1_p_s))
            time.sleep(2)
            saw1_p=end_pos

            print('collision free END?',self.pb.pb_col_free1(saw1_p))
            time.sleep(2)







    def get_random_node(self):
        if random.randint(0, 100) > 20:# self.goalSampleRate:
            #this is where I put my joint limits
            rnd_q=[]
            for i in range(len(self.start_node.q)):

                rnd_q.append(random.uniform(self.jointLimits[0][i],self.jointLimits[1][i]))
                rnd_node=Node(rnd_q)
        else:
            rnd_node=copy.deepcopy(self.end_node)
            #print('end')
            #pdb.set_trace()
        #     rnd_node=Node(rnd_q)
        # else:  # goal point sampling
        #     if random.randint(0, 100) < 100:
        #         rnd_node=copy.deepcopy(self.end_node)
        #
        #     else:
        #         rnd_node=copy.deepcopy(self.start_node)
        #         for i in range(len(rnd_node.q)):
        #             a=random.uniform(-self.expandDis*2,self.expandDis*2)
        #
        #             rnd_node.q[i]=rnd_node.q[i]+a



        return rnd_node

    def closest_node_index(self,node, nodes):
        closest_index = distance.cdist([node], nodes).argmin()
        #print(closest_index,'closest_index')
        return nodes[closest_index]

    def get_nearest_node_index(self,rand_node):


        #print('nodelist',self.nodelist)
        #nodeqlist=[node.q for node in self.nodelist]
        #min_index=self.closest_node_index(rand_node.q,nodeqlist)
        node_q_list=[node.q for node in self.nodelist]
        dlist=self.dist_list(rand_node.q,node_q_list)

        #min_index=dlist.index(min(dlist))
        min_index=np.argmin(dlist)
        #pdb.set_trace()
        return min_index

    def weighted_dist_list(self,nodeq, nodesq):
        weighted_nodesq=[self.weightmat + node for node in nodesq]
        #distancelist = distance.cdist([nodeq], nodesq,'euclidean')
        #weighted_nodesq=nodesq
        distancelist = distance.cdist([nodeq], weighted_nodesq,'euclidean')
        return distancelist[0]
    def dist_list(self,nodeq, nodesq):
        #weighted_nodesq=[self.weightmat + node for node in nodesq]
        #distancelist = distance.cdist([nodeq], nodesq,'euclidean')
        weighted_nodesq=nodesq
        distancelist = distance.cdist([nodeq], weighted_nodesq,'euclidean')
        return distancelist[0]


    def steer(self,nearest_node_idx,rand_node):
        #print(nearest_node_idx)
        #nearestNode=self.nodelist(nearest_node_idx)
        #print('nodelist',self.nodelist,nearest_node_idx)
        nearestNode=self.nodelist[nearest_node_idx]
        new_node=rand_node

        #nearest_node_q=np.array(nearestNode.q)
        node_q_list=[node.q for node in self.nodelist]
        nearest_node_q=node_q_list[nearest_node_idx]
        new_node_q=np.array(new_node.q)
        diff=new_node_q-nearest_node_q
        magnitude=np.linalg.norm(diff)
        #pdb.set_trace()

        if magnitude <= self.expandDis:
            pass
        else:
            new_node_q=nearest_node_q+ self.expandDis*(diff/magnitude)
            new_node.q=new_node_q

        new_node.cost=float('inf')
        new_node.parent=None
        #print('steer', new_node.parent)
        return new_node

    def obstacle_free_path(self,node_start,node_goal):
        def collision_free_joints(obstaclelist,pos):
                for obs in obstaclelist:
                    dx = abs(obs[0] - pos[0])
                    dy = abs(obs[1] - pos[1])
                    d = dx * dx + dy * dy
                    if d <= obs[2] ** 2:
                        return False
        if len(node_start.q)==2:
            q_start=node_start.q
            q_goal=np.array(node_goal.q)
            q_tocheck=[]
            jtoreturn=[]
            valid=1
            for i in range(len(q_start)):
                a1=np.linspace(q_start[i],q_goal[i],50)
                q_tocheck.append(a1.tolist())

            q_tocheck_pos=zip(*q_tocheck)
            sum=0
            for pos in  q_tocheck_pos:
                collision_free=collision_free_joints(self.obstacleList,pos)
                if collision_free==0:
                    sum=sum+1

            if sum !=0:
                valid=0
            else:
                valid=1
            return valid

        else:
            q_start=node_start.q
            q_goal=np.array(node_goal.q)
            q_tocheck=[]
            jtoreturn=[]
            valid=1
            for i in range(len(q_start)):
                a1=np.linspace(q_start[i],q_goal[i],50)
                q_tocheck.append(a1.tolist())

            q_tocheck_pos=zip(*q_tocheck)
            sum=0
            for pos in  q_tocheck_pos:
                in_collision=0
                if self.numsawyers==2:
                    saw1_p=pos[0:7]
                    saw2_p=pos[7:]
                    #pdb.set_trace()
                    collision_free=self.pb.pb_col_free2(saw1_p,saw2_p)
                    if collision_free==0:
                        sum=sum+1
                elif self.numsawyers==1:

                    saw1_p=pos
                    collision_free=self.pb.pb_col_free1(saw1_p)
                    #print(collision_free)
                    if collision_free==0:
                        sum=sum+1
                    #pdb.set_trace()
                    #pdb.set_trace
                    # if in_collision ==1:
                    #
                    #     return 0

            #valid=1
            jtoreturn=q_tocheck_pos
            if sum !=0:
                valid=0
            else:
                valid=1
            return valid





    def collision_free(self,node):
        col_free=0
        if len(node.q)==2:

            for (ox, oy, size) in self.obstacleList:
                dx = abs(ox - node.q[0])
                dy = abs(oy - node.q[1])
                d = dx * dx + dy * dy
                if d <= size ** 2:
                    #pdb.set_trace()
                    return False
            return True
        else:
            q_list=node.q
            if self.numsawyers==2:
                saw1_p=q_list[0:7]
                saw2_p=q_list[7:]
                col_free=self.pb.pb_col_free2(saw1_p,saw2_p)
            elif self.numsawyers==1:
                saw1_p=q_list
                col_free=self.pb.pb_col_free1(saw1_p)

            return col_free


    def near_verticies_index_list(self, new_node):
        nnode=len(self.nodelist)
        #r=self.NearNodeSearchRadius * 5* ((math.log(nnode) / nnode))**(1/len(self.start_node.q))

        if len(new_node.q)==2:
            s=4.18879
        elif len(new_node.q)==14:
            s=.381443
        else:
            s=4.05


        r=((self.expandDis/s)*(math.log(nnode) / nnode))**(1/len(self.start_node.q))
        #print('r',r)

        #nodeqlist=[node.q for node in self.nodelist]
        node_q_list=[node.q for node in self.nodelist]
        dlist=self.weighted_dist_list(new_node.q,node_q_list)
        dtuplist=[]
        for i in range(len(dlist)):
            dtuplist.append((dlist[i],i))

        dtuplist_sorted=sorted(dtuplist, key=lambda x: x[0])


        near_idxs=[]
        if len(dtuplist_sorted)<self.k:
            for i in range(len(dtuplist_sorted)):
                near_idxs.append(dtuplist_sorted[i][1])
        else:
            for i in range(self.k):
                near_idxs.append(dtuplist_sorted[i][1])
        #print(len(self.nodelist),near_idxs)
        #pdb.set_trace()
        return near_idxs

        #r=50.0 * math.sqrt((math.log(nnode) / nnode))
        #r=np.linalg.norm(np.array(dlist))/2
        #r=10
        #r=self.NearNodeSearchRadius
        #print(dlist)
        #near_idxs=[dist_list[0].index(i) for i in dist_list if i <= r ** 2]

        #near_idxs=[dlist.index(i) for i in dist_list if i <= r ** 2]
        #r=.2
        #rad=np.mean(np.array(dlist))/4

        #near_idxs=[i for i,v in enumerate(dlist <= rad) if v]



        #take 5 nearest points


        #pdb.set_trace()
        return near_idxs

    def choose_parent(self,new_node,near_verticies_index_list):

        if len(near_verticies_index_list) ==0:
            return new_node
        #print('CP')
        #pdb.set_trace()
        #near_verticies_index_list is the list of indexes in self.nodelist for the near verticies
        #pdb.set_trace()
        nearNodes=[]
        near_nodes_q_list=[]
        #node_q_list=[node.q for node in self.nodelist]
        for n in near_verticies_index_list:
            v=self.obstacle_free_path(new_node,self.nodelist[n])
            #pdb.set_trace()
            #print('v',v)
            if self.obstacle_free_path(new_node,self.nodelist[n]) ==1:
                #print('sim')
                if self.numsawyers != 0:
                    #self.simulate_movement(new_node, n)
                    pass
                nearNodes.append(self.nodelist[n])
                near_nodes_q_list.append(self.nodelist[n].q)  #gather the near nodes together

        #pdb.set_trace()
        #near_node_q_list=[node.q for node in nearNodes]
        if len(nearNodes)==0:
            #print('no near nodes')
            return new_node
        #print('near nodes')
        near_node_dist_list=self.dist_list(new_node.q,near_nodes_q_list)
        #print('NEARNODES',len(nearNodes),near_node_dist_list)

        cost_list=[]
        for n in range(len(nearNodes)):

            cost_list.append(nearNodes[n].cost+near_node_dist_list[n])
        #print('cost_list',cost_list)
        #pdb.set_trace()
        min_cost=min(cost_list)
        minind=near_verticies_index_list[cost_list.index(min_cost)]
        #pdb.set_trace()
        #print('OBSCHECK',self.obstacle_free_path(new_node,self.nodelist[minind]))
        #pdb.set_trace()
        if min_cost==float('inf'):
            return new_node
        new_node.cost=min_cost
        new_node.parent=minind
        return new_node


    def rewire(self, new_node,near_verticies_index_list):
        nnode=len(self.nodelist)
        nearNodes=[]
        near_node_q_list=[]
        for n in near_verticies_index_list:
            nearNodes.append(self.nodelist[n])
            #near_node_q_list.append(self.node_q_list[n])


        near_node_q_list=[node.q for node in nearNodes]
        new_node_q=np.array(new_node.q)
        for node in nearNodes:
            near_node_q=np.array(node.q)
            diff=new_node_q-near_node_q
            dis=np.linalg.norm(diff)
            #pdb.set_trace()
            scost= new_node.cost + dis
            if node.cost > scost:
                if self.obstacle_free_path(new_node,node):
                    #print(node.parent,node.cost)
                    #pdb.set_trace()
                    node.parent = nnode -1
                    #print('parent',node.parent)
                    node.cost=scost


    def get_best_last_index(self):
        dist_to_goal_list=[self.calc_dist_to_goal(node) for node in self.nodelist]
        #goal_idxs = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.goalTol]
        goal_idxs = []
        for idx in range(len(dist_to_goal_list)):
            if dist_to_goal_list[idx]<=self.goalTol:
                if  self.obstacle_free_path(self.nodelist[idx],self.end_node):
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
           for (ox, oy, size) in self.obstacleList:
               #plt.plot(ox, oy, "ok", ms=30 * size)
               #plt.patches.Circle((ox,oy), radius=size)
               #circle1=plt.Circle((ox,oy),size,color='r')
               #plt.gcf().gca().add_artist(circle1)
               theta = np.linspace(0., 2*3.14)
               x=[]
               y=[]
               for val in theta:
                   x.append(ox+ size*math.cos(val))
                   y.append(oy+size*math.sin(val))
               plt.plot(x,y,'r*')
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
                pass
                plt.plot(node.q,jspacing,'g+')
                #plt.plot([node.q[0], self.nodelist[node.parent].q[0]],[node.q[1], self.nodelist[node.parent].q[1]] , "g-")
            else:
                    #plt.plot(node.q[0],node.q[1],'g+')
                if node.parent is not None:
                    #plt.plot(node.q[0],node.q[1],'g+')
                    plt.plot([node.q[0], self.nodelist[node.parent].q[0]],[node.q[1], self.nodelist[node.parent].q[1]] , "g-")

        plt.axis([-5, 5, -5, 5])
        plt.grid(True)
        plt.pause(0.01)


    def get_linspace_path(self,startpos,goalpos):
        #pdb.set_trace()
        q_start=startpos.q

        q_goal=goalpos.q
        q_ls=[]
        #pdb.set_trace()
        for i in range(len(q_start)):
            a1=np.linspace(q_start[i],q_goal[i],50)
            q_ls.append(a1.tolist())

        q_toret_pos=zip(*q_ls)
        return q_toret_pos

    def simulate_path(self):
        #pdb.set_trace()
        if self.pb is not None:
            self.pb.disconnect()
        if self.show_pb==0:
            if self.numsawyers==2:
                self.pb=pybullet_env(1,self.numsawyers)
                ones=[1,1,1,1,1,1,1]
                zeros=[0,0,0,0,0,0,0]

                print('collision free START?',self.pb.pb_col_free2(zeros,zeros))
                time.sleep(2)


            elif self.numsawyers==1:
                self.pb=pybullet_env(1,self.numsawyers)
                ones=[1,1,1,1,1,1,1]
                zeros=[0,0,0,0,0,0,0]
                print('starttest')

                print('collision free START?',self.pb.pb_col_free1(zeros))
                time.sleep(2)



        ls_fp=[]
        #pdb.set_trace()
        for i in range(len(self.path)-1):
            ls_fp.append(self.get_linspace_path(self.path[i],self.path[i+1]))


        for i in range(len(ls_fp)):
            #pdb.set_trace()
            for pos in  ls_fp[i]:
                #pdb.set_trace()
                #print('pos',pos)
                if self.numsawyers==2:
                    saw1_p=pos[0:7]
                    saw2_p=pos[7:]
                    #pdb.set_trace()
                    #self.pb.move_sawyers_to_joint_pos2(saw1_p,saw2_p)
                    pb_col_free2=self.pb.pb_col_free2(saw1_p,saw2_p)
                elif self.numsawyers==1:
                    saw1_p=pos
                    #pdb.set_trace()
                    pb_col_free1=self.pb.pb_col_free1(saw1_p)
                    print(pb_col_free1)
                    #self.pb.move_sawyers_to_joint_pos1(saw1_p)
                time.sleep(.1)


    def simulate_movement(self,node_start, node_goal):
        #pdb.set_trace()
        if self.pb is not None:
            self.pb.disconnect()
        if self.show_pb==0:
            if self.numsawyers==2:
                self.pb=pybullet_env(1,self.numsawyers)
                ones=[1,1,1,1,1,1,1]
                zeros=[0,0,0,0,0,0,0]



            elif self.numsawyers==1:
                self.pb=pybullet_env(1,self.numsawyers)
                ones=[1,1,1,1,1,1,1]
                zeros=[0,0,0,0,0,0,0]




        ls_fp=[]
        #pdb.set_trace()
        start_pos=node_start.q
        goal_pos=node_goal.q
        path=[start_po]

        for i in range(len(start_pos)):
            a1=np.linspace(start_pos[i],goal_pos[i],50)
            q_tocheck.append(a1.tolist())

        ls_fp=zip(*q_tocheck)

        for i in range(len(ls_fp)):
            #pdb.set_trace()
            for pos in  ls_fp[i]:
                #pdb.set_trace()
                #print('pos',pos)
                if self.numsawyers==2:
                    saw1_p=pos[0:7]
                    saw2_p=pos[7:]
                    #pdb.set_trace()
                    #self.pb.move_sawyers_to_joint_pos2(saw1_p,saw2_p)
                    pb_col_free2=self.pb.pb_col_free2(saw1_p,saw2_p)
                elif self.numsawyers==1:
                    saw1_p=pos
                    #pdb.set_trace()
                    pb_col_free1=self.pb.pb_col_free1(saw1_p)
                    #print(pb_col_free1)
                    #self.pb.move_sawyers_to_joint_pos1(saw1_p)
                time.sleep(.1)
    def main_alg(self):
        self.nodelist.append(self.start_node)

        self.node_q_list.append(np.array(self.start_node.q))

        for i in range(self.numiters):
            if i%10==0:
                print('i',i)
            rand_node=self.get_random_node()
            nearest_node_idx=self.get_nearest_node_index(rand_node)
            #print('nnidx',nearest_node_idx,'lennodelist',len(self.nodelist))
            new_node=self.steer(nearest_node_idx,rand_node)

            if self.collision_free(new_node):
                #print('COLLISION FREE')
                #time.sleep(2)
                if  self.pbsph and self.numsawyers >0:
                    if self.numsawyers==2:
                        ee1=self.pb.get_sawyer_ee_pos('sawyer1')
                        ee2=self.pb.get_sawyer_ee_pos('sawyer2')

                        self.pb.loadurdf(self.spherefile,ee1[0])
                        self.pb.loadurdf(self.spherefile,ee2[0])
                    else:
                        ee1=self.pb.get_sawyer_ee_pos('sawyer1')
                        self.pb.loadurdf(self.spherefile,ee1[0])


                near_node_idx_list=self.near_verticies_index_list(new_node)
                new_node=self.choose_parent(new_node,near_node_idx_list)
                #print("parent")
                #print('parentMAINALG',new_node.parent)
                self.nodelist.append(new_node)
                self.node_q_list.append(new_node.q)

                #self.rewire(new_node, near_node_idx_list)
                if self.drawgraph:
                    self.DrawGraph(rand_node)
                    #pdb.set_trace()

                    # writer.grab_frame()


        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return []
        self.path = self.gen_final_course(lastIndex)

        return self.path




def main():


    sawyer_neutralpos=[0.00, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]
    print("Start rrt planning")

    #Test Case 1
    jl2=[[-3,-3],[3,3]]  #joint limits
    sp2=[0.0,0.0]       #starting point
    ep2=[.4,.4]       #ending point
    wmat2=[1,1]         #weight matrix for joints
    ed2=.2              #expand distance
    nnsr2=3             #nearest node search radius
    gsr2=2              #goal sample rate
    gt2=.1             #goal tolerance
    drawgraph2=1
    ni2=200
    show_pb2=0

    #Test Case 2
    jln=[ [-1,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3,-3.3161],[1,3,3,3,3,3,3,3,3,3,3,3,3,3.3161] ]
    spn=[-1.00, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161,0.00, -1.18, 1.00, 2.18, 0.00, 0.57, 3.3161]
    epn=[0.35,0.0,0.0,0.0,0.0,0.0,0.35,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    wmatn=[700,200,150,100,50,10,1]
    edn=.1
    nnsrn=30
    gsrn=20
    gtn=math.sqrt(.1*14)
    drawgraphn=0
    nin=1000
    show_pbn=1

    #Test Case 3
    jl7=[[-3,-3,-3,-3,-3,-3,-3.5],[3,3,3,3,3,3,3.5]]
    sp7=[-1.00, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161 ]
    ep7=[0.35,0.0,0.0,0.0,0.0,0.0,0.0]
    wmat7=[700,700,150,100,50,10,1]
    ed7=.5
    nnsr7=30
    gsr7=20
    gt7=math.sqrt(.1*7)
    drawgraph7=0
    ni7=1000
    show_pb7=0

    TestCase=3  #1=2d, 2=14dof, 3=7dof

    if TestCase==1:
        jl=jl2
        sp=sp2
        ep=ep2
        Wmat=wmat2
        ed=ed2
        nnsr=nnsr2
        gsr=gsr2
        gt=gt2
        drawgraph=drawgraph2
        ni=ni2
        show_pb=show_pb2
        numsawy=0

    elif TestCase==2:
        jl=jln
        sp=spn
        ep=epn
        wmat=wmatn
        Wmat=wmat+wmat
        ed=edn
        nnsr=nnsrn
        gsr=gsrn
        gt=gtn
        drawgraph=drawgraphn
        ni=nin
        show_pb=show_pbn
        numsawy=2

    elif TestCase==3:
        jl=jl7
        sp=sp7
        ep=ep7
        wmat=wmat7
        Wmat=wmat7
        ed=ed7
        nnsr=nnsr7
        gsr=gsr7
        gt=gt7
        drawgraph=drawgraph7
        ni=ni7
        show_pb=show_pb7
        numsawy=1




    rrts=RRTStarNd( numiters=ni,expandDis=ed, jointLimits=jl,start_pos=sp,end_pos=ep, NearNodeSearchRadius=nnsr,goalSampleRate=gsr,goalTol=gt,WeightMat=Wmat,DrawGraph=drawgraph, ShowPybullet=show_pb, numsawyers=numsawy )
    path = rrts.main_alg()

    attempts_to_find_plan=5


    if path is None or len(path)==0:
        print("Cannot find path")
        notpath=1
        if rrts.pb is not None:
            rrts.pb.disconnect()
    else:
        print("found path!!")
        path_q_list=[node.q for node in path]
        print(path_q_list)
        rrts.simulate_path()
        if len(sp) ==2:

            rrts.DrawGraph()
            x=[item[0] for item in path_q_list]
            y=[item[1] for item in path_q_list]
            plt.plot(x,y,'r-')
            plt.ioff()
            plt.pause(0.1)

            plt.show()
            if rrts.pb is not None:
                rrts.pb.disconnect()

        notpath=0
    count=0

    while notpath and count<attempts_to_find_plan:
        print('count',count)
        rrts=RRTStarNd( numiters=ni,expandDis=ed, jointLimits=jl,start_pos=sp,end_pos=ep, NearNodeSearchRadius=nnsr,goalSampleRate=gsr,goalTol=gt,WeightMat=Wmat,DrawGraph=drawgraph, ShowPybullet=show_pb, numsawyers=numsawy )
        path = rrts.main_alg()
        if (path is None or len(path)==0) :
            print("Cannot find path")
            notpath=1
            if rrts.pb is not None:
                rrts.pb.disconnect()
        else:
            print("found path!!")
            print('NNSR',rrts.NearNodeSearchRadius)
            path_q_list=[node.q for node in path]
            print('')
            print(path_q_list)
            rrts.simulate_path()
            #pdb.set_trace()
            if len(sp) ==2:
                plt.clf()
                rrts.DrawGraph()
                x=[item[0] for item in path_q_list]
                y=[item[1] for item in path_q_list]
                plt.plot(x,y,'r-')
                plt.show()
                notpath=0
            if rrts.pb is not None:
                rrts.pb.disconnect()
        count=count+1

    if rrts.pb is not None:
        rrts.pb.disconnect()


if __name__ == '__main__':
    main()
