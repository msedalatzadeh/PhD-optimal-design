function y=myfun(a,b)
y=integral(@(x)exp(-x.^2),a,b);
end
