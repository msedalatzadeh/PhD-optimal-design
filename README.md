# Optimal Actuator Location for Railway Tracks
Actuator location and design are important design variables  in controller synthesis for distributed parameter systems. Finding the best actuator location to control a distributed parameter system can significantly reduce the cost of the control and improve  its effectiveness. From a theoretical point of view, the existence of an optimal actuator location has been discussed in the literature for linear partial differential equations (PDE's). It was proven that an optimal actuator location exists for linear-quadratic control.  Conditions under which using approximations in optimization yield the optimal location are also established.
 Similar results have been obtained for $H_2$ and $H_\infty$ controller design objectives with linear models. There are also results on the related problem of optimal sensor location for linear PDE's.



## Raiway Track Model
Railway tracks are rested on ballast which are known for exhibiting nonlinear viscoelastic behavior. If a track beam is made of a Kelvin-Voigt material, then the railway track model will be a semi-linear partial differential equation on $\xi\in [0,\ell]$ as follows:

\begin{flalign}\notag
\begin{cases}
\begin{aligned}
&\rho a \frac{\partial^2 w}{\partial t^2}+\frac{\partial}{\partial \xi^2}(EI\frac{\partial^2 w}{\partial \xi^2}+C_d\frac{\partial ^3 w}{\partial \xi^2 \partial t})+\mu \frac{\partial w}{\partial t}+kw+\alpha w^3=b(\xi;r)u(t), &\xi&\in(0,\ell),\\[2mm]
&w(\xi,0)=w_0(\xi), \quad \frac{\partial w}{\partial t}(\xi,0)=v_0(\xi), &\xi&\in(0,\ell),\\[2mm]
&w(0,t)=w(\ell,t)=0,&t&\ge 0,\\[2mm]
&EI\frac{\partial ^2w}{\partial \xi^2}(0,t)+C_d\frac{\partial ^3w}{\partial \xi^2\partial t}(0,t)=EI\frac{\partial^2 w}{\partial \xi^2}(\ell,t)+C_d\frac{\partial ^3w}{\partial \xi^2\partial t}(\ell,t)=0, &t&\ge 0.
\end{aligned}
\end{cases}
\end{flalign}

where the positive constants $E$, $I$, $\rho$, $a$, and $\ell$ are the modulus of elasticity, second moment of inertia, density of the beam, cross-sectional area, and length of the beam, respectively. The linear and nonlinear parts of the foundation elasticity correspond to the coefficients $k$ and $\alpha$, respectively. The constant $\mu\ge 0$ is the viscous damping coefficient of the foundation, and $C_d\ge 0$ is the coefficient of Kelvin-Voigt damping in the beam.
The track deflection is controlled by an external force  $u(t)$;  $u(t)$  will  be
assumed to be a scalar input in order to simplify the exposition. The shape influence function $b(\xi;r)$ is a continuous function over $[0,\ell]$ parametrized by the parameter $r$ that describes its dependence on actuator location. For example, as shown in the next figure, the control force is typically localized at some point $r$ and $b(\xi;r)$ models the distribution of the force $u(t)$ along the beam. The
function $b(\xi;  r) $  is assumed continuously differentiable with respect to $r$ over $\mathbb{R}$; a suitable function for the case of actuator location is illustrated in the figure.

<p align="center">
<img src="figs/Beam-schematic-2.JPG" width="400" />
</p>






## Galerkin Solver
This repository contains codes for solve nonlinear PDE models

```python
import time
tic=time.clock()
import dill
import numpy
from sympy import *
#diff, symbols, simplify, integrate, lambdify, sin, zeros, sqrt, pi
N=10  #Specify the maximum order of approximation
alpha, l, x, r, delta, EI, rhoa, k, par = symbols('alpha l x r delta EI rhoa k\
                                                  par', positive=True)
par=(EI,rhoa,k,l,alpha,delta)
b=(x-r+delta)**2*(x-r-delta)**2/(16*delta**5)*15
br=diff(b,r)
z=symbols('z0:%d'%(N+1))
cc=symbols('cc0:%d'%(N+1))
W=0
for n in range(1,N//2+1):
    print(n)
    W=W+z[2*n-1]*cc[n]*sin(n*pi*x/l)/(n**2*pi**2) \
      -z[2*n]*cc[n]*sin(n*pi*x/l)/(n**2*pi**2)
    F=zeros(2*n,1)
    B=zeros(2*n,1)
    dB=zeros(1,2*n)
    for i in range(1,n+1):
        integrand=expand_trig(W**3*sin(i*pi*x/l))
        F[2*i-2]=-alpha*cc[i]*simplify(integrate(integrand,(x,0,l)))
        F[2*i-1]=F[2*i-2]
      
        B[2*i-2]=cc[i]*simplify(integrate(b*sin(i*pi*x/l),(x,r-delta,r+delta)))
        B[2*i-1]=B[2*i-2]
      
        dB[2*i-2]=cc[i]*simplify(integrate(br*sin(i*pi*x/l),(x,r-delta,r+delta)))
        dB[2*i-1]=dB[2*i-2]
    
    dF=zeros(2*n)
    for i in range(1,n+1):
        for j in range(1,n+1):
            integrand=expand_trig(W**2*sin(i*pi*x/l)*sin(j*pi*x/l))
            dF[2*i-2,2*j-2]=-3*alpha*cc[i]*cc[j]*simplify(\
                    integrate(integrand,(x,0,l)))
            dF[2*i-1,2*j-2]=dF[2*i-2,2*j-2]
            dF[2*i-2,2*j-1]=dF[2*i-2,2*j-2]
            dF[2*i-1,2*j-1]=dF[2*i-2,2*j-2]
    for i in range(1,n+1):
        F=F.subs(cc[i],sqrt((2*l**3*pi**4*i**4)/(EI*pi**4*i**4+k*l**4\
                    +rhoa*l**4*pi**4*i**4)))
        B=B.subs(cc[i],sqrt((2*l**3*pi**4*i**4)/(EI*pi**4*i**4+k*l**4\
                    +rhoa*l**4*pi**4*i**4)))
        dB=dB.subs(cc[i],sqrt((2*l**3*pi**4*i**4)/(EI*pi**4*i**4+k*l**4\
                    +rhoa*l**4*pi**4*i**4)))
        dF=dF.subs(cc[i],sqrt((2*l**3*pi**4*i**4)/(EI*pi**4*i**4+k*l**4\
                    +rhoa*l**4*pi**4*i**4)))

    zz=zeros(2*n,1)
    for i in range(0,2*n):
        zz[i,0]=z[i+1]
        
    FN= lambdify((zz,[*par]),F,dummify=True)
    BN= lambdify((r,[*par]),B,dummify=True)
    dBN= lambdify((r, [*par]),dB,dummify=True)
    dFN= lambdify((zz,[*par]),dF,dummify=True)
    dill.settings['recurse'] = True
    dill.dump(FN,open('F%d'%(2*n),'wb'))
    dill.dump(BN,open('B%d'%(2*n),'wb'))
    dill.dump(dBN,open('dB%d'%(2*n),'wb'))
    dill.dump(dFN,open('dF%d'%(2*n),'wb'))
toc=time.clock()
print('CPUtime=%g'%(toc-tic))
```
