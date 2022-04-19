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
        c=plt.Circle((self.x,self.y),self.radius, color=color, fill=False)
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

        
        Gopher.draw_circle(env.ax,'g')

        env.fig.savefig('./{}.png'.format(str(step)), dpi=100)

        plt.show()
        plt.close()

    def refresh_plot(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(15,15))


    def title(self, title):
        self.ax.set_title(title,loc='center')



def check_velocity(o,gopher, gvel, theta):
    vrelx=gvel*cos(theta)-o.x_vel
    vrely=gvel*sin(theta)-o.y_vel

    relx=o.x-gopher.x
    rely=o.y-gopher.y

    dotproduct= (relx*vrelx+rely*vrely)**2
    r=relx**2+rely**2

    vr=vrelx**2+vrely**2

    rad=(gopher.radius+o.radius)**2


    if(r-(dotproduct/vr)>=rad):
        return True
    else:
        return False


class Velocity_Obstacles():

    def __init__(self, env, begin, end, gopher, obstacles):
        self.env=env
        self.begin=begin
        self.end=end
        self.gopher=gopher
        self.obstacles=obstacles
        self.threshold=2
        self.found_path=False
        self.gen_rand=200
        self.traj= [start]

    def gen_random_vel(self, current):
        theta=np.random.uniform(0,2*np.pi)
        vel=np.random.uniform(0,self.gopher.maximum_vel)

        xnew=current.x+(vel*cos(theta)*dt)
        ynew=current.y+(vel*sin(theta)*dt)

        return vel,theta,Point(xnew,ynew)
    
    def cost_function(self, next):

        cost=np.sqrt((next.return_x()-self.end.return_x())**2+(next.return_y()-self.end.return_y())**2)
        
        return cost

    def velocity_optimizer(self):
        best=self.cost_function(self.traj[-1])
        step=0

        while not self.found_path:
            best_step=None

            for _ in range(self.gen_rand):
                collision_free_step=True

                next_vel,next_theta,next_step=self.gen_random_vel(self.traj[-1])
                for o in self.obstacles:
                    if not check_velocity(o,self.gopher,next_vel, next_theta):
                        collision_free_step=False
                        break

                if collision_free_step:
                    cost=self.cost_function(next_step)
                    if cost<best:
                        best=cost
                        best_step=next_step

            if best_step is not None:
                step+=1
                self.traj.append(best_step)      
                for o in self.obstacles:
                    o.Obs_move()
                self.gopher.robot_move(best_step)

            print(best)
            if best< self.threshold:
                self.found_path=True
           
pranav@pranav-Nitro-AN517-54:~/Desktop/MotionPlanningCodes/Project$ 
minX=0
miny=0
maxX=200
maxY=200


vel_max=5

radius=2


start=Point(0,0)
goal=Point(100,100)

robot=Gopher(start.return_x(),start.return_y(), radius, vel_max)

obstacles=[Obstacle(35, 75, 10, 5, np.pi/6), Obstacle(30, 40, 10, 5, np.pi/3), Obstacle(10, 30, 5 , 5, 0), Obstacle(50, 0, 6 , 1, np.pi/2) ]

env=Environment(minX,miny,maxX,maxY,start,goal)

VO_traj=Velocity_Obstacles(env, start,goal,robot,obstacles)
VO_traj.velocity_optimizer()