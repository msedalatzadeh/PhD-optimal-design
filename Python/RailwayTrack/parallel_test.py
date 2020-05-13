#from multiprocessing import Pool
#import multiprocessing
#from joblib import Parallel, delayed
#import time
#tic=time.clock()
#
#def f(x):
#    return x*x
#
#if __name__ == '__main__':
#    __spec__ = "ModuleSpec(name='builtins', loader=<class '_frozen_importlib.BuiltinImporter'>)"
#    pool=multiprocessing.Pool(2)
#    a=pool.map(f,range(0,10000))
#toc=time.clock()
#print('CPUtime=%g'%(toc-tic))
#%%
from math import sqrt
from joblib import Parallel, delayed
Parallel(n_jobs=2)(delayed(sqrt)(i ** 2) for i in range(10))
