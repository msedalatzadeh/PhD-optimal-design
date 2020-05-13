#def FVP(t,p,x_t,N,An,dFn,Qn,matmul):
#    x=x_t(t)
#    pdot=-matmul(An.T+dFn(x).T,p)-matmul(Qn,x)
#    return pdot

def FVP(t,p,x_t,N,An,dFn,Qn,matmul):
    x=x_t(t).reshape((-1,1))
    pdot=-(An.T+dFn(x).T)@p-Qn@x
    return pdot