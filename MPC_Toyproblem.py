import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
import cvxpy as cp
import time
dt=1
class Point():

    def __init__ (self,x,y):
        self.x=x
        self.y=y

    def return_x(self):
        return self.x

    def return_y(self):
        return self.y 

    def draw_point(self,ax,color,marker,size):
        ax.scatter(self.x,self.y,c=color, marker=marker,s=size)



class Circle():
    def __init__(self,x,y,r):
        self.x=x 
        self.y=y 
        self.radius=r 


    def return_x(self):
        return self.x 

    def return_y(self):
        return self.y 

    def return_r(self):
        return self.radius

    def draw_circle(self,ax,color):
        c=plt.Circle((self.x,self.y),self.radius, color=color, fill=True)
        ax.add_patch(c)



class Gopher(Circle):
    def __init__(self, x,y,r,vel):
        super().__init__(x,y,r)
        self.maximum_vel=vel
        self.robot_path=[]
        self.robot_path.append([self.x,self.y])
        self.path_length=0

    def robot_move(self, next_pt):
        self.x=next_pt.x 
        self.y=next_pt.y 

        self.robot_path.append([self.x,self.y])

    def path_length_calculation(self, next_pt):
        self.path_length+=((self.x-next_pt.x)**2+(self.y-next_pt.y)**2)**0.5
        


    
class Obstacle(Circle):

    def __init__(self, x,y,r,vel,theta):

        super().__init__(x,y,r)
        
        self.x_vel=vel*cos(theta)
        self.y_vel=vel*sin(theta)


    def Obs_move(self):
        self.x+= self.x_vel*dt
        self.y+=self.y_vel*dt



class Environment():

    def __init__(self,minX, minY, maxX, maxY, begin, end):
        self.mX=minX
        self.mY=minY
        self.MX=maxX 
        self.MY=maxY

        self.start=begin
        self.end=end

        self.fig, self.ax = plt.subplots(1, 1, figsize=(15,15))


    def draw_Environment(self, Gopher, obs,step):
        self.refresh_plot()

        self.title('Model Predictive Control')
        self.start.draw_point(self.ax,'g','*',100)
        self.end.draw_point(self.ax,'r','*',100)

        for o in obs:
            o.draw_circle(env.ax,'r')

        
        Gopher.draw_circle(env.ax,'b')

        env.fig.savefig('./{}.png'.format(str(step)), dpi=100)

        #plt.show()
        #plt.close()

    def refresh_plot(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(15,15))


    def title(self, title):
        self.ax.set_title(title,loc='center')


def distance(x1,y1,x2,y2):
    return ((x1-x2)**2+(y1-y2)**2)**0.5

class MPC:
    def __init__(self,env, gopher,obstacles, start,goal):
        self.env=env
        self.gopher=gopher
        self.obstacles=obstacles
        self.start=start
        self.goal=goal
        self.threshold=5
        self.horizon=1
        self.max=self.gopher.maximum_vel
        self.solthresh=5
        self.min_clear=np.inf
        
    def constraints(self, X, Y, obs):
        fx=((self.gopher.return_x()-obs.return_x()+X.sum()*dt)**2+(self.gopher.return_y()-obs.return_y()+Y.sum()*dt))**2-(self.gopher.return_r()+obs.return_r())**2
        delf= (np.array([2*dt*(self.gopher.return_x() + X.sum() * dt - obs.return_x())]*len(X)), np.array([2*dt*(self.gopher.return_y() + Y.sum() * dt - obs.return_y())]*len(Y)))
        return fx,delf


    def MPC_path_planner(self):
        step=0
        #then=time.time()
        total_time=0
        while distance(self.gopher.return_x(),self.gopher.return_y(),self.goal.return_x(),self.goal.return_y())>self.threshold:
            xd,yd=np.zeros(shape=self.horizon), np.zeros(shape=self.horizon)
            Xd,Yd=cp.Variable(shape=(self.horizon,1)),cp.Variable(shape=(self.horizon,1))
            
            prev_sol=np.inf
            while True:
                cons=[]

                for s in range(self.horizon):
                    cons.extend([-self.max<=Xd[s],Xd[s]<=self.max, -self.max<=Yd[s], Yd[s]<=self.max])


                for o in self.obstacles:
                    dist=distance(self.gopher.x,self.gopher.y,o.x,o.y)
                    self.min_clear=min(dist,self.min_clear)
                    fx,delf=self.constraints(xd[:s+1],yd[:s+1],o)
                    cons.append(fx+delf[0].reshape(1,-1)@(Xd[:s+1]-xd[:s+1].reshape(-1,1))+delf[1].reshape(1,-1)@(Yd[:s+1]-yd[:s+1].reshape(-1,1))>=0)

                optimize=cp.Minimize((self.gopher.return_x()-self.goal.return_x()+cp.sum(Xd))**2+(self.gopher.return_y()-self.goal.return_y()+cp.sum(Yd))**2)

                Definition=cp.Problem(optimize,cons)

                solution=Definition.solve()

                if abs(solution-prev_sol)<=self.solthresh:
                    break
                prev_sol=solution

                xd=Xd.value.flatten()
                yd=Yd.value.flatten()
            

            X_next=self.gopher.return_x()+Xd.value[0]
            Y_next=self.gopher.return_y()+Yd.value[0]

            next=Point(X_next,Y_next)
            
            self.gopher.path_length_calculation(next)
            self.gopher.robot_move(next)
            print(self.gopher.path_length)
            
            step+=1

            for o in self.obstacles:
                o.Obs_move()
            self.env.draw_Environment(self.gopher,self.obstacles,step)
            total_time+=dt
            
        print(self.min_clear)

        print(total_time)

            








minX=0
miny=0
maxX=200
maxY=200


vel_max=5
radius=2

obstacles=[]



start=Point(0,0)
goal=Point(100,100)

robot=Gopher(start.return_x(),start.return_y(), radius, vel_max)

#obstacles=[Obstacle(35, 75, 5, 1, np.pi/6), Obstacle(30, 40, 5, 0, np.pi/3), Obstacle(10, 30, 5 , 1, 0), Obstacle(50, 50, 6 , 0.05, np.pi/2) ]


#obstacles=[Obstacle(35, 75, 0.5, 0.0, np.pi/6), Obstacle(30, 40, 2, 0.00, np.pi/3), Obstacle(10, 30, 2 , 0.0, 0), Obstacle(50, 50, 2 , 0, np.pi/2), Obstacle(80, 50, 2, 0, np.pi/6), Obstacle(60, 80, 2, 0, np.pi/3), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 0.001, np.pi/2)]
obstacles=[Obstacle(35, 75, 2, 2, 0), Obstacle(30, 40, 2, 2, 0), Obstacle(10, 30, 2 , 2, 0), Obstacle(50, 50, 2 , 2, 0), Obstacle(80, 50, 2, 2, 0), Obstacle(60, 80, 2, 1, 0), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 1, 0)]

#obstacles=[Obstacle(35, 75, 5, 0.03, 0), Obstacle(30, 40, 5, 0.5, 0), Obstacle(10, 30, 5 , 0.3, 0), Obstacle(50, 50, 6 , 0.7, 0), Obstacle(40, 60, 6 , 0.2, 0), Obstacle(80, 80, 6 , 0.0, 0) ]
#obstacles=[Obstacle(35, 75, 0.5, 0.0, np.pi/6), Obstacle(30, 40, 0.5, 0.00, np.pi/3), Obstacle(10, 30, 0.5 , 0.0, 0), Obstacle(50, 50, 0.5 , 0, np.pi/2), Obstacle(80, 50, 2, 0, np.pi/6), Obstacle(60, 80, 2, 0, np.pi/3), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 0.001, np.pi/2)]


#obstacles=[Obstacle(35, 75, 0.5, 2, 0), Obstacle(30, 40, 0.5, 2, 0), Obstacle(10, 30, 0.5 , 2, 0), Obstacle(50, 50, 0.5 , 2, 0), Obstacle(80, 50, 0.5, 2, 0), Obstacle(60, 80, 0.5, 1, 0), Obstacle(10, 20, 0.5 , 0.001, 0), Obstacle(10,90, 2 , 1, 0)]


env=Environment(minX,miny,maxX,maxY,start,goal)

mpc=MPC(env, robot ,obstacles,start,goal)
mpc.MPC_path_planner()
