#def IVP(t,x,u_t,N,An,Fn,Bn,matmul):
#    u=u_t(t)
#    xdot=matmul(An,x)+Fn(x)[:,0]+u*Bn[:,0]
#    return xdot

def IVP(t,x,u_t,N,An,Fn,Bn,matmul):
    u=u_t(t)
    xdot=An@x+Fn(x)+Bn*u
    return xdot