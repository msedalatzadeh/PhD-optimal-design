#def RTgrad(ur,matrices,x0,N,mesh,T,IVP,FVP,solve_ivp,interp1d,trapz,dot,append,tspan,matmul,inner):
#    u=ur[0:mesh]
#    u_t=interp1d(tspan,u,kind='linear')
#    r=ur[mesh]
#    An=matrices[0]
#    Bn_r=matrices[1]
#    Bn=Bn_r(r)
#    Fn=matrices[2]
#    dBn_r=matrices[3]
#    dBn=dBn_r(r)
#    dFn=matrices[4]
#    Qn=matrices[5]
#    Rn=matrices[6]
#    #%%
#    IVPlam=lambda t, x: IVP(t,x,u_t,N,An,Fn,Bn,matmul)
#    sol_IVP=solve_ivp(IVPlam,[0,T],x0,method='Radau',dense_output=True,\
#                  events=None,rtol=1e-5, atol=1e-6)
#    z_t=sol_IVP.sol
#    #%%
#    FVPlam=lambda t, p: FVP(t,p,z_t,N,An,dFn,Qn,matmul)
#    sol_FVP=solve_ivp(FVPlam,[T,0],[0]*N,method='Radau',t_eval=tspan[::-1],\
#                  events=None,rtol=1e-5, atol=1e-6)
#    p=sol_FVP.y
#    #%%
#    DuJ=tspan*0
#    for k in range(0,mesh):
#        DuJ[k]=2*inner(Bn[:,0],p[:,mesh-k-1])+2*Rn*u[k]
#
#    DrJ=tspan*0
#    for k in range(0,mesh):
#        DrJ[k]=2*u[k]*inner(dBn[0,:],p[:,mesh-k-1])
#    DrJ=trapz(DrJ,tspan)
#    #%%
#    return append(DuJ,DrJ)


def RTgrad(ur,matrices,x0,N,mesh,T,IVP,FVP,solve_ivp,interp1d,trapz,dot,append,tspan,matmul,inner):
    u=ur[0:mesh]
    u_t=interp1d(tspan,u,kind='linear')
    r=ur[mesh]
    An=matrices[0]
    Bn_r=matrices[1]
    Bn=Bn_r(r)
    Fn=matrices[2]
    dBn_r=matrices[3]
    dBn=dBn_r(r)
    dFn=matrices[4]
    Qn=matrices[5]
    Rn=matrices[6]
    #%%
    IVPlam=lambda t, x: IVP(t,x,u_t,N,An,Fn,Bn,matmul)
    sol_IVP=solve_ivp(IVPlam,[0,T],x0,method='RK45',dense_output=True,\
                  events=None,vectorized=True,rtol=1e-5, atol=1e-6)
    z_t=sol_IVP.sol
    #%%
    FVPlam=lambda t, p: FVP(t,p,z_t,N,An,dFn,Qn,matmul)
    sol_FVP=solve_ivp(FVPlam,[T,0],x0*0,method='RK45',t_eval=tspan[::-1],\
                  events=None,vectorized=True,rtol=1e-5, atol=1e-6)
    p=sol_FVP.y
    #%%
    DuJ=tspan*0
    for k in range(0,mesh):
        DuJ[k]=2*dot(Bn.T,p[:,mesh-k-1])+2*Rn*u[k]

    DrJ=tspan*0
    for k in range(0,mesh):
        DrJ[k]=2*u[k]*dot(dBn,p[:,mesh-k-1])
    DrJ=trapz(DrJ,tspan)
    #%%
    return append(DuJ,DrJ)