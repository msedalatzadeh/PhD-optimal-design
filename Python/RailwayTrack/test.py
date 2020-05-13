import pickle
import dill
import numpy
import sympy
from numpy import matrix

x=symbol('x')

#F2 = open("F2","rb")
#FN = dill.load(F2)
#FN(matrix([[1],[2]]),[0.1,0.2,0.3,4,0.5,0.6])

B2 = open("B2","rb")
BN = dill.load(B2)
print(BN(1/2,[0.1,0.2,0.3,4,0.5,0.6]))

dB2 = open("dB2","rb")
dBN = dill.load(dB2)
print(dBN(1/2,[0.1,0.2,0.3,4,0.5,0.6]))

dF2 = open("dF2","rb")
dFN = dill.load(dF2)
print(dFN([1,2],[0.1,0.2,0.3,4,0.5,0.6]))
#%%
import dill
from sympy import *
x=Symbol('x')
F= lambdify(x,sin(x))
dill.settings['recurse'] = True
Fun = open("F2","wb")
dill.dump(F, Fun)
Fun.close()


