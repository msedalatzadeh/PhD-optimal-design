def c(n,par):
    EI=par[1]
    rhoa=par[2]
    k=par[3]  
    l=par[4]
    return ((2*l**3*3.141592653589793**4*n**4)/(EI*3.141592653589793**4*\
            n**4+k*l**4+rhoa*l**4*3.141592653589793**4*n**4))^0.5