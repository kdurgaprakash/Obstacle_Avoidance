import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

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

    def robot_move(self, next_pt):
        self.x=next_pt.x 
        self.y=next_pt.y 

        self.robot_path.append([self.x,self.y])

    
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

        self.title('Velocity obstacle- Toy Problem')
        self.start.draw_point(self.ax,'g','*',100)
        self.end.draw_point(self.ax,'r','*',100)

        for o in obs:
            o.draw_circle(env.ax,'r')

        
        Gopher.draw_circle(env.ax,'b')

        env.fig.savefig('./{}.png'.format(str(step)), dpi=100)

        plt.show()
        plt.close()

    def refresh_plot(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(15,15))


    def title(self, title):
        self.ax.set_title(title,loc='center')


def distance(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2)


class APF():


    def __init__(self, gopher,obstacles, begin, end):
        self.start=begin
        self.goal=end
        self.gopher=gopher
        self.k_att=0.1
        self.k_rep=100
        self.max_dist=20
        self.max_iteration=1000
        self.i=0
        self.thresh=1
        self.path=[]
        self.path_found=False

    def attraction(self):
        attrative_force=np.array([(self.goal.return_x-self.gopher.return_x)*k_att, (self.goal.return_y-self.gopher.return_y)*k_att])
        return attractive_force

    def repulsive(self):
        repulsive_force=np.array([0,0])
        for o in self.obstacles:
            d=dist(self.gopher.return_x,self.gopher.return_y,o.return_x,o.return_y)
            if d>max_dist:
                continue
            else:
                repulsive_force+= 
           

minX=0
miny=0
maxX=200
maxY=200


vel_max=5

radius=2


start=Point(0,0)
goal=Point(100,100)

robot=Gopher(start.return_x(),start.return_y(), radius, vel_max)

obstacles=[Obstacle(35, 75, 10, 0, np.pi/6), Obstacle(30, 40, 10, 0, np.pi/3), Obstacle(10, 30, 5 , 0, 0), Obstacle(50, 50, 6 , 0, np.pi/2) ]

env=Environment(minX,miny,maxX,maxY,start,goal)

VO_traj=Velocity_Obstacles(env, start,goal,robot,obstacles)
VO_traj.velocity_optimizer()