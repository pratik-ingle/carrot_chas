import matplotlib.pyplot as plt 
import math
import numpy as np
from scipy.spatial import distance

locationX = []
locationY = []

g = [0,0]
S = [50,20]
G = [100,100]
k = 0.1
dt = 0.01
v = 1
si= np.pi *2         #intial angle of bot
Rmin= 3*v
umin = -(v**2/Rmin)
umax = (v**2/Rmin)

x = [G[0],g[0]]
y = [G[1],g[1]]
plt.plot(x,y)

#slope of line 
m = (G[1]-g[1])/(G[0]-g[0])

theta = math.atan2(G[1]-g[1], G[0]-g[0])
p = theta 
'''#ang_wrap function
def ang_wrap(p):
    if (p <= np.pi):
        p = p + (2 * np.pi) 
        return p
    if (p > np.pi):
        p = p - (2 * np.pi)
        return p
theta_c = ang_wrap(p)'''

#intial carrot
x = abs(-1*(-1*S[0]+m*S[1])/(((-m)**2)+((-1)**2))) + 2
y = abs((m*(-1)*S[0]+((m**2)*S[1]))/(((-m)**2)+((-1)**2))) +2

locationX.append(S[0])
locationY.append(S[1])

while True:
    

    thetad = math.atan2((y-S[1]),(x-S[0]))


    u = k*(thetad-si)
    if(umin >= u ):
        u = umin
    if(u>umax):
        u= umax
    
    
    si = (si + u*dt)
    
    x = x + dt*math.cos(theta) 
    y = y + dt*math.sin(theta)
    
    #new carrot
    S[0] = S[0] + (v*math.cos(si)*dt)
    S[1] = S[1] + (v*math.sin(si)*dt)
    
    
    locationX.append(S[0])
    locationY.append(S[1])
   
    if((((S[0]-G[0])**2+(S[1]-G[1])**2)**(1/2))<5):
       break
   
    
plt.plot(locationX,locationY)
    
    
    
    
    
    
    
    
    
    
    