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

        self.title('APF- Toy Problem')
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


class APF():


    def __init__(self,env, gopher,obstacles, begin, end):
        self.start=begin
        self.env=env
        self.goal=end
        self.obstacles=obstacles
        self.gopher=gopher
        self.k_att=0.03
        self.k_rep=1000
        self.max_dist=120
        self.max_iteration=1000
        self.i=0
        self.thresh=5
        self.path=[]
        self.path_found=False

    def attraction(self):
        attractive_force=np.array([(self.goal.return_x()-self.gopher.return_x())*self.k_att, (self.goal.return_y()-self.gopher.return_y())*self.k_att])
        return attractive_force

    def repulsive(self):
        repulsive_force=np.array([0,0])
        for o in self.obstacles:
            d=np.array([self.gopher.return_x(),self.gopher.return_y()])-np.array([o.return_x(),o.return_y()])
            u_v=d/np.linalg.norm(d)
            if np.linalg.norm(d)>self.max_dist:
                continue
            else:
                repulsive_force=repulsive_force+ np.array([u_v[0],u_v[1]])*self.k_rep*((1/np.linalg.norm(d))-(1/self.max_dist))/(np.linalg.norm(d)**2)
        return repulsive_force

    def force_calculation(self):

        step=0
        total_time=0
        min_clear=np.inf
        while (self.i < self.max_iteration) and (distance(self.gopher.return_x(),self.gopher.return_y(),self.goal.return_x(),self.goal.return_y())> self.thresh):
            f=self.attraction()+self.repulsive()
            f_vector=f/np.linalg.norm(f)
            next_x=self.gopher.return_x()+(f_vector[0]*dt)
            next_y=self.gopher.return_y()+(f_vector[1]*dt)
            next_step=Point(next_x,next_y)  
            self.path.append([next_x,next_y])
            step+=1
            for o in self.obstacles:
                dist=distance(self.gopher.x,self.gopher.y,o.x,o.y)
                min_clear=min(dist,min_clear)
                o.Obs_move()
            self.gopher.path_length_calculation(next_step)
            self.gopher.robot_move(next_step)
            total_time+=dt
            if distance(self.gopher.return_x(),self.gopher.return_y(),self.goal.return_x(),self.goal.return_y())<self.thresh:
                self.path_found=True
                
            self.env.draw_Environment(self.gopher,self.obstacles,step)
        print(total_time)
        print(self.gopher.path_length)
        print(min_clear)
minX=0
miny=0
maxX=200
maxY=200


vel_max=5

radius=2


start=Point(0,0)
goal=Point(100,100)

robot=Gopher(start.return_x(),start.return_y(), radius, vel_max)

# obstacles=[Obstacle(35, 75, 5, 1, np.pi/6), Obstacle(30, 40, 5, 0, np.pi/3), Obstacle(10, 30, 5 , 1, 0), Obstacle(50, 50, 6 , 0.05, np.pi/2) ]
#obstacles=[Obstacle(35, 75, 2, 0.01, np.pi/6), Obstacle(30, 40, 2, 0.001, np.pi/3), Obstacle(10, 30, 2 , 1, 0), Obstacle(50, 50, 2 , 0, np.pi/2), Obstacle(80, 50, 2, 0, np.pi/6), Obstacle(60, 80, 2, 0, np.pi/3), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 0.001, np.pi/2)]
#obstacles=[Obstacle(35, 75, 2, 0.01, 0), Obstacle(30, 40, 2, 0.001, 0), Obstacle(10, 30, 2 , 1, 0), Obstacle(50, 50, 2 , 0, 0), Obstacle(80, 50, 2, 0, 0), Obstacle(60, 80, 2, 0, 0), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 0.001, 0)]

#obstacles=[Obstacle(35, 75, 2, 0, 0), Obstacle(30, 40, 2, 0, 0), Obstacle(10, 30, 2 , 0, 0), Obstacle(50, 50, 2 , 0, 0), Obstacle(80, 50, 2, 0, 0), Obstacle(60, 80, 2, 0, 0), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 0, 0)]

#obstacles=[Obstacle(35, 75, 2, 2, 0), Obstacle(30, 40, 2, 2, 0), Obstacle(10, 30, 2 , 2, 0), Obstacle(50, 50, 2 , 2, 0), Obstacle(80, 50, 2, 2, 0), Obstacle(60, 80, 2, 1, 0), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 1, 0)]


#Scenario 3
obstacles=[Obstacle(35, 75, 5, 0.03, 0), Obstacle(30, 40, 5, 0.5, 0), Obstacle(10, 30, 5 , 0.3, 0), Obstacle(50, 50, 6 , 0.7, 0), Obstacle(80, 80, 6 , 0.0, 0), Obstacle(40, 60, 6 , 0.2, 0) ]


env=Environment(minX,miny,maxX,maxY,start,goal)

Artificial=APF(env, robot ,obstacles,start,goal)
Artificial.force_calculation()
