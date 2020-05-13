#matrices1=[An,Bn1,Fn,dBn1,dFn,Qn,Rn]
#[z,p,Cost,DuJ,DrJ]=RTzpcostgrad(u1_t,r1,matrices1,z0,N,T,IVP,FVP,solve_ivp,matmul\
#,transpose,len,trapz)

T=10
mesh=300
tspan=linspace(0,T,mesh)
u1=1*cos(0.1*tspan*pi/T)
u1=numpy.ndarray.tolist(u1)
r1=0.5*l;
u1_t=interp1d(tspan,u1,kind='linear')
ur1=[*u1,r1]
#%%
import scipy.io as spio
mat = spio.loadmat('../../ToPython.mat', squeeze_me=True)
tspan=mat['tspan']
u=mat['u_fmin']
r=mat['r_fmin']
mesh=1000
T=10
ur1=[*u,r]

print(RTcost(ur1,matrices2,z0,mesh,T,IVP,solve_ivp,matmul,len,trapz,interp1d\
            ,linspace))

Grad=RTgrad(ur1,matrices2,z0,N,mesh,T,IVP,FVP,solve_ivp,matmul,transpose,len\
            ,interp1d,trapz,linspace)
#plt.plot(numpy.ndarray.tolist(tspan),Grad[0:mesh])
plt.plot(numpy.ndarray.tolist(tspan),Grad[0:mesh])
#%%
import dill
from sympy import Symbol, lambdify, Array
x = Symbol("x")
H = lambdify(Array([x]), x,modules='numpy')
dill.settings['recurse'] = True
H2=dill.dumps(H)
_H = dill.loads(H2)
