restoredefaultpath
savepath
ver('distcomp')                  %this tells you whether it is _installed_
license('test','Distrib_Computing_Toolbox')   %this tells you whether it is _licensed_
distcomp.feature('LocalUseMpiexec',true)
parcluster
feature('numcores')
parpool(20)