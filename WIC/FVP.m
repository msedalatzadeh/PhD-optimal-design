function pdot=FVP(t,p,z,An,dFn,Qn)
pdot=-An'*p-dFn'*p-Qn*z;
end