#def RTcost(ur,matrices,x0,N,mesh,T,IVP,solve_ivp,trapz,interp1d,tspan,matmul,inner):
#    u=ur[0:mesh]
#    u_t=interp1d(tspan,u,kind='linear')
#    r=ur[mesh]
#    An=matrices[0]
#    Bn_r=matrices[1]
#    Bn=Bn_r(r)
#    Fn=matrices[2]
#    Qn=matrices[5]
#    Rn=matrices[6]
#    #%%
#    IVPlam=lambda t, x: IVP(t,x,u_t,N,An,Fn,Bn,matmul)
#    sol_IVP=solve_ivp(IVPlam,[0,T],x0,method='Radau',t_eval=tspan,\
#                  events=None,rtol=1e-5, atol=1e-6)
#    x=sol_IVP.y
#    #%%
#    L=tspan*0
#    for k in range(1,mesh):
#        L[k]=L[k]+inner(x[:,k],matmul(Qn,x[:,k]))+u[k]*Rn*u[k]
#    Cost=trapz(L,tspan)
#    return Cost

def RTcost(ur,matrices,x0,N,mesh,T,IVP,solve_ivp,trapz,interp1d,tspan,matmul,inner):
    u=ur[0:mesh]
    u_t=interp1d(tspan,u,kind='linear')
    r=ur[mesh]
    An=matrices[0]
    Bn_r=matrices[1]
    Bn=Bn_r(r)
    Fn=matrices[2]
    Qn=matrices[5]
    Rn=matrices[6]
    #%%
    IVPlam=lambda t, x: IVP(t,x,u_t,N,An,Fn,Bn,matmul)
    sol_IVP=solve_ivp(IVPlam,[0,T],x0,method='RK45',t_eval=tspan,\
                  events=None,vectorized=True,rtol=1e-5, atol=1e-6)
    x=sol_IVP.y
    #%%
    L=tspan*0
    for k in range(1,mesh):
                L[k]=L[k]+x[:,k].T@Qn@x[:,k]+u[k]*Rn*u[k]
    Cost=trapz(L,tspan)
    return Cost