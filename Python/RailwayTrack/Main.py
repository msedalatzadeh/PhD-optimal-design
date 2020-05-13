from numpy import matmul, eye, transpose, sin, linspace, pi, array, cos, dot, append, multiply, inner
import numpy as np
from scipy.integrate import solve_ivp, trapz
from IVP import IVP
from FVP import FVP
from Qn_op import Qn_op
from RTcost import RTcost
from RTgrad import RTgrad
from state_op import state_op
from scipy.linalg import block_diag
from scipy.interpolate import interp1d
import dill
import time
from scipy.optimize import minimize
import matplotlib.pyplot as plt
#from drawnow import drawnow
#%%
N=6
mu_exp=-1
Cd_exp=-4
EI=1
rhoa=1
k=1
l=1
Alpha=10
delta=0.001/2
mu=10**mu_exp;
Cd=10**Cd_exp;
cw=1
cv=1
cu=1
par=[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu]
x0=np.array([2,3,2/2,3/2,2/4,3/4,2/8,3/8,2/16,3/16])
#x0=[2,3,2/2,3/2,2/4,3/4,2/8,3/8,2/16,3/16]
x0=x0[0:N]
T=10
mesh=500
tspan=linspace(0,T,mesh)
u1=sin(tspan*pi/T)
u1=np.ndarray.tolist(u1)
r1=0.9*l;
u1_t=interp1d(tspan,u1,kind='linear')
#%%
An=np.array(state_op(N,par,block_diag))

Bn = dill.load(open("B%d"%N,"rb"))
Bn1=Bn(r1,par[0:6])
Bn_r=lambda rr: Bn(rr,par[0:6])

F_z = dill.load(open("F%d"%N,"rb"))
Fn=lambda zz: F_z(zz,par[0:6])

dBn = dill.load(open("dB%d"%N,"rb"))
dBn1=dBn(r1,par[0:6])
dBn_r=lambda rr: dBn(rr,par[0:6])

dF_z = dill.load(open("dF%d"%N,"rb"))
dFn=lambda zz: dF_z(zz,par[0:6])

Qn=np.array(Qn_op(N,par,block_diag))
Rn=cu

matrices2=[An,Bn_r,Fn,dBn_r,dFn,Qn,Rn]
ur1=[*u1,r1]
#ini=array(ur1)
#%%
#Cost=RTcost(ur1,matrices2,x0,N,mesh,T,IVP,solve_ivp,trapz,interp1d,linspace)
#Grad= RTgrad(ur1,matrices2,x0,N,mesh,T,IVP,FVP,solve_ivp,interp1d,trapz\
#            ,linspace,dot,append)
#plt.plot(np.ndarray.tolist(tspan),Grad[0:mesh])
#plt.plot(numpy.ndarray.tolist(tspan),p[0][mesh::-1])


func=lambda ur: RTcost(ur,matrices2,x0,N,mesh,T,IVP,solve_ivp,trapz\
                       ,interp1d,tspan,matmul,inner)
jacob=lambda ur: RTgrad(ur,matrices2,x0,N,mesh,T,IVP,FVP,solve_ivp\
                        ,interp1d,trapz,dot,append,tspan,matmul,inner)
jacob_arr=lambda ur: array(RTgrad(ur,matrices2,x0,N,mesh,T,IVP,FVP,solve_ivp\
                        ,interp1d,trapz,dot,append,tspan,matmul,inner))
bnds=(*mesh//2*((-10,10),(-10,10)),(delta,1-delta))

#tic=time.clock()
#resTNC=minimize(func,ur1, method='TNC',jac=jacob, bounds=bnds,\
#                options={'scale': [*mesh*[1],1],'maxiter': 50,'rescale': -1})
#print('>TNC Method:')
#print('ro=%g'%resTNC.x[mesh])
#print('Cost=%g'%resTNC.fun)
toc1=time.clock()
#print('CPUtime=%g'%(toc1-tic))
#p1,=plt.plot(np.ndarray.tolist(tspan),resTNC.x[0:mesh], label='TNC')
#plt.legend(handles=[p1])

resLBFGSB=minimize(func,ur1,method='L-BFGS-B',jac=jacob_arr,bounds=bnds,\
                options={'maxiter': 20})
print('>L-BFGS-B Method:')
print('ro=%g'%resLBFGSB.x[mesh])
print('Cost=%g'%resLBFGSB.fun)
toc2=time.clock()
print('CPUtime=%g'%(toc2-toc1))
p2,=plt.plot(np.ndarray.tolist(tspan),resLBFGSB.x[0:mesh], label='L-BFGS-B')
plt.legend(handles=[p2])

#resSLSQP=minimize(func,ur1,method='SLSQP',jac=jacob, bounds=bnds,\
#                options={'maxiter': 50})
#print('>SLSQP Method:')
#print('ro=%g'%resSLSQP.x[mesh])
#print('Cost=%g'%resSLSQP.fun)
#toc3=time.clock()
#print('CPUtime=%g'%(toc3-toc2))
#p3,=plt.plot(np.ndarray.tolist(tspan),resSLSQP.x[0:mesh],label='SLSQP')
#plt.legend(handles=[p3])

