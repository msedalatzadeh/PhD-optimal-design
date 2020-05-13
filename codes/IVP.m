function zdot=IVP(t,z,u,An,Fn,Bn)
zdot=An*z+Fn+Bn*u;
end