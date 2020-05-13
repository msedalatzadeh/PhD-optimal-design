from numpy import array, matmul
from scipy.integrate import solve_ivp
from numpy import size
def sys(t,x,A,xeq,matmul,size):
    if size(x)==4:
        print(x)
    xdot=matmul(A,x)
    return xdot

T=10
A=array([[-1,2],[-2,-6]])
x0=array([13.67999999,-15.989999999])
xeq=array([[0.99999999],[2.999999999]])
fun=lambda t, x: sys(t,x,A,xeq,matmul,size)
sol=solve_ivp(fun,[0,T],x0,method='BDF',vectorized=True)
#%%
from numpy import array, dot
from scipy.integrate import solve_ivp
from numpy import size
def sys(t,x,A,xeq,dot,size,array):
    x=array(x).reshape((-1,1))
    if size(x)==4:
        print(x)
    xdot=dot(A,x-xeq).reshape((1,-1))[0]
    return xdot

T=10
A=array([[-1,2],[-2,-6]])
x0=[13.67999999,-15.989999999]
xeq=array([[0.99999999],[2.999999999]])
fun=lambda t, x: sys(t,x,A,xeq,dot,size,array)
sol=solve_ivp(fun,[0,T],x0,method='BDF')
#%%
from numpy import array
from scipy.integrate import solve_ivp
def sys(t,x):
    import numpy
    print(x)
    print(numpy.shape(x))
    xdot=array([[-x[1,0]],[-2*x[0,0]]])
    return xdot
T=1
x0=array([13.99999,15.333333])
A=array([[-1,2],[-2,-6]])
sol=solve_ivp(sys,[0,T],x0,method='BDF',vectorized=True)

#%%

#import dill

#T=1 
#N=2
#mu_exp=-1
#Cd_exp=-4
#EI=1
#rhoa=1
#k=1
#l=1
#Alpha=10
#delta=0.001/2
#mu=10**mu_exp;
#Cd=10**Cd_exp;
#cw=1
#cv=1
#cu=1
#par=[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu]
#
#F2= dill.load(open("F2","rb"))
