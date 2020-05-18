# Optimal Controller and Actuator Design for Nonlinear Systems
Actuator location and design are important design variables  in controller synthesis for distributed parameter systems. Finding the best actuator location to control a distributed parameter system can significantly reduce the cost of the control and improve  its effectiveness. From a theoretical point of view, the existence of an optimal actuator location has been discussed in the literature for linear partial differential equations (PDE's). It was proven that an optimal actuator location exists for linear-quadratic control.  Conditions under which using approximations in optimization yield the optimal location are also established.
 Similar results have been obtained for $H_2$ and $H_\infty$ controller design objectives with linear models. There are also results on the related problem of optimal sensor location for linear PDE's. Here, we study the optimal controller and actuator design for a specific model, namely, railway track model. The theoretical framework developed in my thesis will be used. For full details, please download my thesis [here](https://uwspace.uwaterloo.ca/handle/10012/15147). 



## Raiway Track Model
Railway tracks are rested on ballast which are known for exhibiting nonlinear viscoelastic behavior. 

<p align="center">
<img src="figs/track.png" width="600" />
</p>

If a track beam is made of a Kelvin-Voigt material, then the railway track model will be a semi-linear partial differential equation on $\xi\in [0,\ell]$ as follows:

\begin{flalign}\notag
\begin{cases}
\begin{aligned}
&\rho a \frac{\partial^2 w}{\partial t^2}+\frac{\partial}{\partial \xi^2}(EI\frac{\partial^2 w}{\partial \xi^2}+C_d\frac{\partial ^3 w}{\partial \xi^2 \partial t})+\mu \frac{\partial w}{\partial t}+kw+\alpha w^3=b(\xi;r)u(t), &\xi&\in(0,\ell),\\[2mm]
&w(\xi,0)=w_0(\xi), \quad \frac{\partial w}{\partial t}(\xi,0)=v_0(\xi), &\xi&\in(0,\ell),\\[2mm]
&w(0,t)=w(\ell,t)=0,&t&\ge 0,\\[2mm]
&EI\frac{\partial ^2w}{\partial \xi^2}(0,t)+C_d\frac{\partial ^3w}{\partial \xi^2\partial t}(0,t)=EI\frac{\partial^2 w}{\partial \xi^2}(\ell,t)+C_d\frac{\partial ^3w}{\partial \xi^2\partial t}(\ell,t)=0, &t&\ge 0.
\end{aligned}
\end{cases}
\end{flalign}

where the positive constants $E$, $I$, $\rho$, $a$, and $\ell$ are the modulus of elasticity, second moment of inertia, density of the beam, cross-sectional area, and length of the beam, respectively. The linear and nonlinear parts of the foundation elasticity correspond to the coefficients $k$ and $\alpha$, respectively. The constant $\mu\ge 0$ is the viscous damping coefficient of the foundation, and $C_d\ge 0$ is the coefficient of Kelvin-Voigt damping in the beam.
The track deflection is controlled by an external force  $u(t)$;  $u(t)$  will  be
assumed to be a scalar input in order to simplify the exposition. The shape influence function $b(\xi;r)$ is a continuous function over $[0,\ell]$ parametrized by the parameter $r$ that describes its dependence on actuator location. For example, as shown in the next figure, the control force is typically localized at some point $r$ and $b(\xi;r)$ models the distribution of the force $u(t)$ along the beam. The
function $b(\xi;  r) $  is assumed continuously differentiable with respect to $r$ over $\mathbb{R}$; a suitable function for the case of actuator location is illustrated in the figure.

<p align="center">
<img src="figs/Beam-schematic-2.JPG" width="400" />Fig. 1
</p>













## Optimal Controller and Actuator Location
In this section, we apply the results of previous sections to compute an optimal control and actuator location for the vibration suppression of the track. As discussed in chapter 3, the problem of finding the best control and actuator location is the optimization problem
\begin{equation*}
\begin{cases}
\min&J(u,r;x_0)\\
\text{s.t.}&\dot{x}(t)=Ax(t)+F(x(t))+B(r)u(t), \quad \forall t\in(0,T],\\
&x(0)=x_0
\end{cases}
\end{equation*}

The optimality conditions use the derivative of the cost function with respect to the input and the actuator location. In that, the adjoint of the IVP needs to be calculated. The adjoint equation is the following final value problem (FVP):
\begin{equation*}
\dot{p}(t)=-(A^*+F'^*_x(t))p(t)-Qx(t),\quad p(T)=0
\end{equation*}

For every $x_0\in X$, the derivatives of the cost function with respect to $u$ and $r$ evaluated at $u\in \text{int}(U_{ad})$, $r\in \text{int}(K_{ad})$ are linear operators $D_u J(u,r;x_0)$ and $D_{r}J(u,r;x_0)$, respectively. Identifying each operator with an element of its underlying space, they are derived as

\begin{equation*}
\begin{cases}
D_u J(u,r;x_0)&=B^*(r)p(t)+R u(t),\\
D_{r}J(u,r,x_0)&=\int_0^T(B'_{r}u(s))^*p(s)\, ds.
\end{cases}
\end{equation*}





















## Optimization Algorithms
Several optimization algorithms were suggested in the literature for solving PDE-constrained optimization problems. In this section, two common optimization algorithms for solving the optimization problem will be discussed. These are projected gradient method and nonlinear conjugate gradient method. In projected gradient (or steepest descent) method, the negative of the gradient is considered as the search direction. 

#### Projected Gradient Algorithm
The projected gradient method reads as follows:

\begin{enumerate}
\item \textbf{input} initial guesses $u_1\in U_{ad}$ and $r_1\in K_{ad}$
\item Set $n=1$
\item \textbf{while} a criteria is met \textbf{do}
\item \hspace*{0.5cm}Solve the IVP with $u_n$ and $r_n$, and find $x_n$
\item \hspace*{0.5cm}Solve the FVP with $x_n$, and find $p_n$
\item \hspace*{0.5cm}Evaluate $d^u_n:= -D_u J(u_n,r_n,x_0) $ and $d^r_n:= -D_{r}J(u_n,r_n;x_0)$ from previous section
\item \hspace*{0.5cm}Obtain step lengths $s^u_n$ and $s^r_n$ by one of the line search methods discussed below  
\item \hspace*{0.5cm}Set $u_{n+1}=u_n+s_n^u d_n^u $ and $r_{n+1}=r_n+s_n^rd_n^r$
\item \hspace*{0.5cm}increase $n$
\item \textbf{end while}
\end{enumerate}

Projected gradient method is typically converging to an optimizer slowly, whereas the nonlinear conjugate gradient method promises faster convergence. 

#### Conjugate Gradient Algorithm
The nonlinear conjugate gradient method reads as follows:

\begin{enumerate}
\item \textbf{input} initial guesses $u_1\in U_{ad}$ and $r_1\in K_{ad}$

\item Set $n=1$

\item Set $d^u_n= h^u_n:= -D_u J(u_n,r_n,x_0)$ and $d^r_n= h^r_n:=-D_{r}J(u_n,r_n;x_0)$

\item \textbf{while} a criteria is met \textbf{do}

\item \hspace*{0.5cm}Solve the IVP for $u_n$ and $r_n$, and find $x_n$

\item \hspace*{0.5cm}Solve the FVP for $x_n$, and find $\pb_n$

\item \hspace*{0.5cm}Obtain step lengths $s^u_n$ and $s^r_n$ using, e.g., secant method

\item \hspace*{0.5cm}Set $u_{n+1}=u_n+s_n^u d_n^u $ and $r_{n+1}=r_n+s_n^r d_n^r$

\item \hspace*{0.5cm}Solve the IVP for $u_{n+1}$ and $r_{n+1}$, and find $x_{n+1}$

\item \hspace*{0.5cm}Solve the FVP for $x_{n+1}$, and find $\pb_{n+1}$

\item \hspace*{0.5cm}Evaluate $h^u_{n+1}:= -D_u J(u_{n+1},r_{n+1},x_0) $ and $h^r_{n+1}:=-D_{r}J(u_{n+1},r_{n+1};x_0)$ using previous section

\item \hspace*{0.5cm}Determine step lengths $\beta^u_{n+1}$ and $\beta^r_{n+1}$ using, e.g., Fletcher-Reeves or Polak-Ribi\`ere formula

\item \hspace*{0.5cm}Set $d^u_{n+1}:= h^u_{n+1}+\beta^u_{n+1}d^u_{n}$ and $d^r_{n+1}:= h^r_{n+1}+\beta^r_{n+1}d^r_{n}$

\item \hspace*{0.5cm}\textbf{if} $\langle d^u_{n+1}}{h^u_{n+1}}_{L^2(0,T;U)}\le 0$ \textbf{then}

\item \hspace*{1cm}Set $d^u_{n+1}=h^u_{n+1}$

\item \hspace*{0.5cm}\textbf{end if}

\item \hspace*{0.5cm}\textbf{if} $\langle d^r_{n+1}}{h^r_{n+1}}_{K}\le 0$ \textbf{then}

\item \hspace*{1cm}Set $d^r_{n+1}=h^r_{n+1}$

\item \hspace*{0.5cm}\textbf{end if}

\item \hspace*{0.5cm}increase $n$

\item \textbf{end while}
\end{enumerate}



Several choices exist for selecting the step length $\beta^u_{n+1}$ (similarly $\beta^r_{n+1}$) of the previous algorithm. Letting $\gamma^u_{n+1}=h^u_{n+1}-h^u_n$, the following are considered for selecting the step length $\beta^u_{n+1}$ (similarly $\beta^r_{n+1}$)

\begin{flalign*}
&\text{Fletcher-Reeves:} \quad \beta^u_{n+1}=\frac{\| h^u_{n+1}\|_{U}}{\| h^u_n\|_{U}},\\
&\text{Polan-Ribi\`ere:} \quad \beta^u_{n+1}=\frac{\langle h^u_{n+1} , \gamma^u_{n+1} \rangle_U}{\| h^u_n\|_{U}},\\
&\text{Hestenes-Stiefel:} \quad  \beta^u_{n+1}=\frac{\langle h^u_{n+1} , \gamma^u_{n+1} \rangle_U}{\langle d^u_n , \gamma^u_{n+1} \rangle_U}.
\end{flalign*}



A new formula was also proposed by Hager and Zhang; define $\bar{\beta}^u_{n+1}$ and $\eta^u_{n+1}$ as

\begin{flalign}
\bar{\beta}^u_{n+1}&=\frac{\langle \gamma^u_{n+1}-2\frac{\| \gamma^u_{n+1} \|_U^2}{\langle d^u_{n} , \gamma^u_{n+1} \rangle_U}d^u_n , h^u_{n+1} \rangle_U}{\langle d^u_{n+1} , \gamma^u_{n+1} \rangle_U},\notag \\
\eta^u_{n+1}&=-\frac{1}{\| d^u_n \|_{U}}\min\left\{0.01,\| h^u_n \|_{U}\right\}\notag.
\end{flalign}



Then, the formula is
\begin{equation*}
\text{Hager-Zhang:}\quad \beta^u_{n+1}=\max\left\{\bar{\beta}^u_{n+1},\eta^u_{n+1}\right\}
\end{equation*}




Furthermore, several schemes have been proposed to choose the step length $s^u_n$ (similarly $s^r_n$) in each iteration of previous algorithms including bisection, (strong) Wolfe conditions, Secant method.

\begin{enumerate}
\item \textbf{Bisection:} In each iteration $n$ of the algorithms, initialize $s^u_{n,1}$ and $s^r_{n,1}$. Set $m=1$. Compute $u_{n,m}=u_{n}+s^u_{n,m} d^u_{n}$, $r_{n,m}=r_{n}+s^r_{n,m} d^r_{n}$, and $J(u_{n,m},r_{n,m};x_0)$. If
\begin{equation*}
J(u_{n,m},r_{n,m};x_0)\le J(u_{n},r_n;x_0),
\end{equation*}
then accept the step size; otherwise, set $s^u_{n,m+1}=\frac{1}{2}s^u_{n,m}$ and $s^r_{n,m+1}=\frac{1}{2}s^r_{n,m}$ and repeat the process.

\item \textbf{Wolfe conditions:} In each iteration $n$ of the algorithms, initialize $s^u_{n,1}$ and $s^r_{n,1}$. Set $m=1$. Pick constants $c_1$ and $c_2$ in the interval $(0,1)$. Compute $u_{n,m}=u_{n}+s^u_{n,m} d^u_{n}$ and $r_{n,m}=r_{n}+s^r_{n,m} d^r_{n}$ together with  $h^u_{n,m}=-D_u J(u_{n,m},r_{n,m};x_0)$ and $h^r_{n,m}=-D_r J(r_{n,m},r_{n,m};x_0)$. Iterate the step size $s^u_{n,m}$ and $s^r_{n,m}$ until the following conditions are met

\begin{flalign*}
J(u_{n,m},r_n;x_0)&\le J(u_{n},r_n;x_0)+c_1s^u_{n,m}\langle h^u_{n,m} , d^u_{n} \rangle_U\\
&\quad +c_1s^r_{n,m}\langle h^r_{n,m} , d^r_{n}\rangle_K,\\
\langle h^u_{n,m} , d^u_{n} \rangle_U&\ge c_2  \langle h^u_{n} , d^u_{n} \rangle_U,\\
\langle h^r_{n,m} , d^r_{n} \rangle_K&\ge c_2  \langle h^r_{n} , d^r_{n} \rangle_K.
\end{flalign*}

\item \textbf{Strong Wolfe conditions:} Similar to Wolfe conditions except the previous condition is replaced with

\begin{flalign*}
|\langle h^u_{n,m+1} , d^u_{n,m} \rangle_U|&\le c_2  |\langle h^u_{n,m} , d^u_{n,m} \rangle_U|,\\
|\langle h^r_{n,m+1} , d^r_{n,m} \rangle_K|&\le c_2  |\langle h^r_{n,m} , d^r_{n,m} \rangle_K|.
\end{flalign*}

\item The step lengths can be approximate minimizers of the function $\theta(s^u ,s^r):= J(u_n+s^u d^u_n,r_n+s^rd^r_n;x_0)$. For instance, letting $\sigma^u $ and $\sigma^r$ be some positive constants, an approximate minimizer of $\theta(s^u ,s^r)$ can be derived by using secant formula as

\begin{equation*}
s^u =\frac{\theta_{s^u }(0,0)}{\theta_{s^u }(\sigma^u ,0)-\theta_{s^u }(0,0)},\; s^r=\frac{\theta_{s^r}(0,0)}{\theta_{s^r}(0,\sigma^r)-\theta_{s^r}(0,0)},
\end{equation*}

where the subscripts indicate partial derivatives. In the first iteration, the constants $\sigma^u $ and $\sigma^r$ are chosen arbitrary; in next iterations, they are set to the values of $s^u $ and $s^r$ found in the previous iteration. Accordingly, from the definition of $\theta(s^u ,s^r)$, and by arbitrary initializing $s^u_0$ and $s^r_0$, it follows that 

\begin{flalign*}
s^u_n &=-\frac{\langle h^u_n, d^u_n \rangle_U}{\langle h^u_n+D_u J(u_n+s^u_{n-1} d^u_n,r_n;x_0) , d^u_n \rangle_U},\\ 
s^r_n &=-\frac{\langle h^r_n , d^r_n \rangle_K}{\langle h^r_n+D_r J(u_n,r_n+s^r_{n-1} d^r_n;x_0) , d^u_n \rangle_K}.
\end{flalign*}

\end{enumerate}















## Simulation Results

In the railway track model, if the variables $w$, $t$, and $\xi$ are appropriately substituted with dimensionless variables, a dimensionless PDE model for the railway track can be derived. Moreover, the final interval time is set to $\tau=10$. This gives the state of the controlled system enough time to settle. In addition, we choose $\delta=5\times 10^{-5}$ so that the input force is concentrated on a relatively small region on the track. In the cost function, we choose the same weights for the deflection, rate of deflection, and input; so $c_w=c_v=c_u=1$ is selected for the simulations. 

Given an order of approximation, the initial conditions are chosen such that all modes are excited. The initial conditions are chosen from
\begin{equation*}
x_0=\left(2,\, 3,\, \frac{2}{2},\, \frac{3}{2},\, \frac{2}{4},\, \frac{3}{4},\, \frac{2}{8},\, \frac{3}{8},\, \frac{2}{16},\, \frac{3}{16}\right).
\end{equation*}

The order of approximation is equal to the dimension of an initial condition. For example, if the order of approximation is 4, the initial condition is

\begin{equation*}
x_0=\left(2,\, 3,\, \frac{2}{2},\, \frac{3}{2}\right).
\end{equation*}

The initial condition is illustrated in for the 10th order approximation. 

<p align="center">
<img src="results/initinal-condition.jpg" width="400" />
</p>


#### Programming
Simulations were conducted using the software MATLAB, Python, and C++. The programming is carried out extensively. The first code that was created `Generator.m` uses MATLAB Symbolic Toolbox to find the symbolic discretization of the railway track PDE and return a symbolic set of ordinary differential equations (ODEs). The script later uses `MatlabFunction` to generate matlab function that are optimized for next computation.

```matlab
%% Generator.m
syms alpha l x r delta EI rhoa k l par
assume(alpha>0 & l>0 & r>0 & EI>0 & rhoa>0 & k>0 & l>0 & delta>0)
assumeAlso(2*delta<l & delta<r & r+delta<l)
par=[EI,rhoa,k,l,alpha,delta];

b=piecewise(r-delta<x<r+delta,(x-r+delta)^2*(x-r-delta)^2/(16*delta^5)*15,0);
br=diff(b,r)

for n=1:N
   z=[z;sym(sprintf('z%d',n))];
end
for n=1:N/2
   cc=[cc,sym(sprintf('c%d',n))];
   W=W+z(2*n-1)*cc(n)*sin(n*pi*x/l)/(n^2*pi^2)...
      -z(2*n)*cc(n)*sin(n*pi*x/l)/(n^2*pi^2);
   for i=1:n
      F(2*i-1,1)=-alpha*cc(i)*simplify(int(W^3*sin(i*pi*x/l),x,0,l));
      F(2*i,1)=F(2*i-1,1);

.
.
. %% Please see the original file in the repository
.
.

      
      B(2*i-1,1)=cc(i)*simplify(int(b*sin(i*pi*x/l),x,0,l));
   dB=subs(dB,cc,c(1:n,par));
   
   matlabFunction(dB,'File',sprintf('dB%d',2*n),'Vars',{r,par});
toc
```

We later use the m-functions to build a system of ODEs. The module `RTsolver.m` solves the railway track PDE given an input, actuator location, and parameters of the model.  The ODE solver `ode15s` was used to solve the finite-dimensional approximation of the system.

```matlab
%% RTsolver.m
function [z,p,DuJ,DrJ,Cost]=RTsolver(ur,par)

F=str2func(sprintf('F%d',n));
B=str2func(sprintf('B%d',n));
dF=@(z)dF(z,par{1});
dB=@(r)dB(r,par{1});

options=odeset('RelTol',1e-5,'AbsTol',1e-6);
sol_IVP=ode15s(@(t,z)IVP(t,z,interp1(tspan,u,t),A,F(z),B(r)),tspan,z0,options);
sol_FVP=ode15s(@(t,p)FVP(t,p,deval(sol_IVP,t),A,dF(deval(sol_IVP,t)),Q),fliplr(tspan),0*z0,options);
z=deval(sol_IVP,tspan);
p=deval(sol_FVP,tspan);


.
.
. %% Please see the original file in the repository
.
.



DuJ=2*(B(r)'*p+R*u);
DrJ=zeros(1,m);
Cost=trapz(tspan,L);
end
```

We later use the solve inside an optimization scheme to fild the best control and actuator location. MATLAB optimization routine `fmincon` was used as the optimization algorithm. This task is executed in `ResultsGeenrator.m` that evetually saves the results as a `.mat` file in a directory.

```matlab
%% ResultsGeenrator.m
u1=sin(tspan*pi/T);
r1=0.9*l;
z0=z0(1:N)
par={[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
%% Optimizer uo and ro
disp('Optimizer uo and ro')
options=optimoptions('fmincon','Display','off','SpecifyObjectiveGradient'...
   ,true,'HonorBounds',true,'MaxIterations',35,'Algorithm',...
   'interior-point','SubproblemAlgorithm','cg','HessianApproximation','bfgs');
rmin=0+par{1}(6);
rmax=par{1}(4)-par{1}(6);
lb=[-10*ones(1,length(tspan)),rmin];
ub=[+10*ones(1,length(tspan)),rmax];

[ur,Cost]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[]...
   ,options);
cputime=toc;
m=length(tspan);
u_fmin=ur(1:m);
r_fmin=ur(m+1);


.
.
. %% Please see the original file in the repository
.
.


while i<=mesh
    for n=1:N/2
        w0(i)=w0(i)+z0(2*n-1)*c(n,par{1})*sin(n*pi*x(i)/l)/(n^2*pi^2)...
            -z0(2*n)*c(n,par{1})*sin(n*pi*x(i)/l)/(n^2*pi^2);
        v0(i)=v0(i)+z0(2*n-1)*c(n,par{1})*sin(n*pi*x(i)/l)...
            -z0(2*n)*c(n,par{1})*sin(n*pi*x(i)/l);
    end
    i=i+1;
end
save(sprintf('%gmode&Alpha%g&Cd%g&mu%g.mat',N,Alpha,Cd_exp,mu_exp))
```


Finally, the script `Plotter.m` is created to produce readable and informative plots.

```matlab
%% Plotter.m
set(groot,'defaultLineLineWidth',1)
set(0,'defaultAxesFontSize',14)
set(groot,'defaulttextinterpreter','latex')
set(groot,'defaultAxesTickLabelInterpreter','latex')
set(groot,'defaultLegendInterpreter','latex');

%% Optimal Input vs Time
hu=figure;
%set(h,'LineColor','red')
plot(tspan,uo,'Color','black')
title('Optimal input')
n=length(un)
for i=1:n
   hold on
   plot(tspan,un{i})
   drawnow
   pause(0.5)
end
plot(tspan,u_lqr,'--')
plot(tspan,u_fmin)
%% Optimal Cost
hc_comp=figure
plot(r,J_fmin,r,J_lqr)
legend('Nonlinear model','Linear model')

.
.
. %% Please see the original file in the repository
.
.

```

Since the computation is heavy and involves long runs, we executed the codes on SHARCNET supercomputers and incorporated parallel programming on up to 21 cores. The following bash script is used to submit the jobs

```bash
#!/bin/bash
# bash script for submitting a Matlab job to the sharcnet Graham queue

#SBATCH --mem-per-cpu=1G        # memory per processor (default in Mb)
#SBATCH --time=00-02:00         # time (DD-HH:MM)
#SBATCH --job-name="Sajjad"     # job name
#SBATCH --input=ResultsGenerator6.m   # Matlab script
##SBATCH �dependency=afterok:<jobid>  # Wait for job to complete

#SBATCH --ntasks=1              # number of processors
#SBATCH --cpus-per-task=15
#SBATCH --output=log1.log                 # log file
#SBATCH --error=errorfile.err                  # error file
#SBATCH --mail-user=FAIL     # who to email
#SBATCH --mail-type=FAIL                    # when to email
#SBATCH --account=def-kmorris              # UW Fluids designated resource allocation
module load matlab/2018a 
srun matlab -nodisplay -nosplash
```


The convergence of the approximation method is illustrated. It is observed that beyond 6th order approximation, increasing the approximation order will not make a noticeable difference. The following figure compare the cost and optimal input for the linear and nonlinear model in the presence and absence of damping. These figures indicate a significant change in the cost of control and in the optimal input. It also shows how the cost and optimal location of actuators change when the coefficient of nonlinearity, $\alpha$, is increased. As a general rule of thumb, increasing $\alpha$ increases the cost of control. Moreover, it shows how the cost and location of actuators change when the coefficient of viscous and Kelvin-Voigt damping are decreased. Simulations show that the optimal location of actuators moves away from the center as the damping is decreased. Also, an interesting observation is made where local optimizers appear by decreasing the coefficient of Kelvin-Voigt damping. Lastly, it shows the improvement in the performance of the control system when the optimal location is chosen for the actuator over the center location.  


<p align="center">
<img src="results/increasing-order.png" width="400" />

Convergence of the numerical scheme for different orders of approximation in undamped beam. No significant improvement is observed for 4th order approximation or higher.
</p>





<p align="center">
<img src="results/increasing-order-undamped.png" width="400" />

Convergence of the numerical scheme for different orders of approximation in undamped beam. No significant improvement is observed for 6th order approximation or higher.
</p>





<p align="center">
<img src="results/linearVsnonlinear-damping.png" width="400" />

Comparison of the optimal input and cost function in linear and nonlinear damped beam. The cost of control increases by increasing the nonlinearity.
</p>







<p align="center">
<img src="results/linearVSnonlinear-nodamping.png" width="400" />

Comparison of the optimal input and cost function in linear and nonlinear undamped beam. The cost of control increases by increasing the nonlinearity.
</p>






<p align="center">
<img src="results/alpha.png" width="400" />

Effect of nonlinearity on the cost function. The optimal actuator locations do not change significantly despite the change in the cost.
</p>






<p align="center">
<img src="results/mu.png" width="400" />

Effect of viscous damping on the cost function. The optimal actuator locations move away from center as the damping is decreased.
</p>







<p align="center">
<img src="results/Cd.png" width="400" />

Effect of Kelvin-Voigt damping on the cost function. If $C_d=0$, the beam models is hyperbolic. The optimal actuator locations move away from center as the damping is decreased.
</p>






<p align="center">
<img src="results/optimalVSmidpoint.png" width="400" />

Comparison of optimal inputs: optimal location vs center. Actuators on optimal locations improve the control input.
</p>


## References
[1] **M. S. Edalatzadeh** and K. A. Morris, “Optimal actuator design for semilinear systems,” SIAM Journal of Control & Optimization, vol. 57, no. 4, pp. 2992–3020, 2019.

[2] **M. S. Edalatzadeh** and K. A. Morris, “Stability and Well-posedness of a Nonlinear Railway Track Model,” IEEE Control System Letters, vol. 3, no. 1, pp. 162–167, 2019.

[3] **M. S. Edalatzadeh** and K. A. Morris, “Optimal controller and actuator design for nonlinear parabolic systems,” Submitted. available on arXiv 1903.07572, 2019.

[4] **M. S. Edalatzadeh**, D. Kalise, K. A. Morris, and K. Sturm, “Optimal actuator design for vibration control based on LQR performance and shape calculus,” Submitted. available on arXiv 1903.07572, 2019.

[5] R. Herzog and K. Kunisch, “Algorithms for PDE-constrained optimization,” GAMM Mitteilungen, vol. 33, no. 2, pp. 163–176, 2010.

[6] J. Nocedal and S. J. Wright, Numerical optimization. Springer-Verlag New York, 2006.


[7] R. Buchholz, H. Engel, E. Kammann, and F. Tröltzsch, “On the optimal control of the Schlögl-model,” Computational Optimization and Applications, vol. 56, no. 1, pp. 153–185, Sep. 2013.

[8] W. W. Hager and H. Zhang, “A new conjugate gradient method with guaranteed descent and an efficient line search,” SIAM Journal of Optimization, vol. 16, no. 1, pp. 170–192, 2005.

[9] W. W. Hager and H. Zhang, “A survey of nonlinear conjugate gradient methods,” Pacific Journal of Optimization, vol. 2, no. 1, pp. 35–58, 2006.
