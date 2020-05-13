def RTzpcostgrad(u,r,matrices,z0,N,T,IVP,FVP,solve_ivp,matmul,transpose,len,trapz):
    An=matrices[0]
    Bn=matrices[1]
    Fn=matrices[2]
    dBn=matrices[3]
    dFn=matrices[4]
    Qn=matrices[5]
    Rn=matrices[6]
    #%%
    IVPlam=lambda t, z: IVP(t,z,u,An,Fn,Bn,matmul)
    sol_IVP=solve_ivp(IVPlam,[0,T],z0,method='RK45',dense_output=True,\
                  events=None,vectorized=True)
    t_IVP=sol_IVP.t
    z=sol_IVP.y
    z_t=sol_IVP.sol
    #%%
    FVPlam=lambda t, p: FVP(t,p,z_t,An,dFn,Qn,matmul,transpose)
    sol_FVP=solve_ivp(FVPlam,[T,0],[0]*N,method='RK45',dense_output=True,\
                  events=None,vectorized=True)
    
    t_FVP=sol_FVP.t
    p=sol_FVP.y
    #%%
    n=len(t_IVP)
    L=n*[0]
    for i in range(1,n):
        L[i]=matmul(z[:,i].reshape((1,-1)),matmul(Qn,z[:,i].reshape((-1,1))))\
        +u(t_IVP[i])*Rn*u(t_IVP[i])
    Cost=trapz(L,t_IVP)
    #%%
    n=len(t_FVP)
    DuJ=n*[0]
    for i in range(0,n):
        Array2D=2*matmul(p[:,i].reshape((1,-1)),Bn)+Rn*u(t_FVP[i])
        DuJ[i]=Array2D[0][0]

    DrJ=n*[0]
    for i in range(0,n):
        DrJ[i]=2*u(t_FVP[i])*sum((dBn[:]*p[:,i]).ravel())
    DrJ=trapz(DrJ,t_FVP)
    #%%
    return [z,p,Cost,DuJ,DrJ]