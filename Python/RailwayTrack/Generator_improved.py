import time
tic=time.clock()
import dill
from sympy import *
#diff, symbols, simplify, integrate, lambdify, sin, zeros, sqrt, pi
N=4  #Specify the maximum order of approximation
alpha, l, x, r, delta, EI, rhoa, k, par = symbols('alpha l x r delta EI rhoa k\
                                                  par', positive=True)
par=(EI,rhoa,k,l,alpha,delta)
b=(x-r+delta)**2*(x-r-delta)**2/(16*delta**5)*15
br=diff(b,r)
W=0
for n in range(1,N//2+1):
    z=symbols('z0:%d'%(2*n+1))
    cc=symbols('cc0:%d'%(2*n+1))
    print(n)
    W=W+z[2*n-1]*cc[n]*sin(n*pi*x/l)/(n**2*pi**2) \
      -z[2*n]*cc[n]*sin(n*pi*x/l)/(n**2*pi**2)
    F=2*n*[0,]  
    B=2*n*[0,]
    dB=2*n*[0,]
    for i in range(1,n+1):
        integrand=expand_trig(W**3*sin(i*pi*x/l))
        F[2*i-2]=-alpha*cc[i]*integrate(integrand,(x,0,l))
        for ii in range(1,n+1):
            F[2*i-2]=F[2*i-2].subs(cc[ii],sqrt((2*l**3*pi**4*ii**4)/(EI*pi**4*ii**4+k*l**4\
                    +rhoa*l**4*pi**4*ii**4)))
        F[2*i-1]=F[2*i-2]
      
        B[2*i-2]=sqrt((2*l**3*pi**4*i**4)/(EI*pi**4*i**4+k*l**4\
                    +rhoa*l**4*pi**4*i**4))\
         *simplify(integrate(b*sin(i*pi*x/l),(x,r-delta,r+delta)))
        B[2*i-1]=B[2*i-2]
      
        dB[2*i-2]=sqrt((2*l**3*pi**4*i**4)/(EI*pi**4*i**4+k*l**4\
                    +rhoa*l**4*pi**4*i**4))\
          *simplify(integrate(br*sin(i*pi*x/l),(x,r-delta,r+delta)))
        dB[2*i-1]=dB[2*i-2]
    
    dF=[[0]*2*n for i in range(2*n)]
    for i in range(1,n+1):
        for j in range(1,n+1):
            integrand=expand_trig(W**2*sin(i*pi*x/l)*sin(j*pi*x/l))
            dF[2*i-2][2*j-2]=-3*alpha*cc[i]*cc[j]*integrate(integrand,(x,0,l))
            for ii in range(1,n+1):
                dF[2*i-2][2*j-2]=dF[2*i-2][2*j-2].subs(cc[ii],sqrt((2*l**3*pi**4*ii**4)/(EI*pi**4*ii**4+k*l**4\
                    +rhoa*l**4*pi**4*ii**4)))
            dF[2*i-1][2*j-2]=dF[2*i-2][2*j-2]
            dF[2*i-2][2*j-1]=dF[2*i-2][2*j-2]
            dF[2*i-1][2*j-1]=dF[2*i-2][2*j-2]

    
    F=simplify(F)
    #B=simplify(B)
    #dB=simplify(dB)
    dF=simplify(dF)
    
#    F=ImmutableDenseNDimArray(F,(2*n,1))
#    B=ImmutableDenseNDimArray(B,(2*n,1))
#    dB=ImmutableDenseNDimArray(dB,(1,2*n))
#    dF=ImmutableSparseNDimArray(dF,(2*n,2*n))
#        
#    zz=MutableDenseNDimArray(2*n*(0,),(2*n,1))
#    for i in range(0,2*n):
#        zz[i,0]=z[i+1]
    
    FN= lambdify((*z[1:],*par),F,modules='math',dummify=True)
    BN= lambdify((r,*par),B,modules='math',dummify=True)
    dBN= lambdify((r, *par),dB,modules='math',dummify=True)
    dFN= lambdify((*z[1:],*par),dF,modules='math',dummify=True)
    dill.settings['recurse'] = True
    dill.dump(FN,open('F%d'%(2*n),'wb'))
    dill.dump(BN,open('B%d'%(2*n),'wb'))
    dill.dump(dBN,open('dB%d'%(2*n),'wb'))
    dill.dump(dFN,open('dF%d'%(2*n),'wb'))
toc=time.clock()
print('CPUtime=%g'%(toc-tic))