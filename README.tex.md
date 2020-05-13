# Optimal Actuator Location for Railway Tracks
Actuator location and design are important design variables  in controller synthesis for distributed parameter systems. Finding the best actuator location to control a distributed parameter system can significantly reduce the cost of the control and improve  its effectiveness. From a theoretical point of view, the existence of an optimal actuator location has been discussed in the literature for linear partial differential equations (PDE's). It was proven that an optimal actuator location exists for linear-quadratic control.  Conditions under which using approximations in optimization yield the optimal location are also established.
 Similar results have been obtained for $H_2$ and $H_\infty$ controller design objectives with linear models. There are also results on the related problem of optimal sensor location for linear PDE's.



## Raiway Track Model
Railway tracks are rested on ballast which are known for exhibiting nonlinear viscoelastic behavior. If a track beam is made of a Kelvin-Voigt material, then the railway track model will be a semi-linear partial differential equation on $\xi\in [0,\ell]$ as follows:

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
<img src="figs/Beam-schematic-2.JPG" width="400" />
</p>













## Optimal Controller and Actuator Location
In this section, we apply the results of previous sections to compute an optimal control and actuator location for the vibration suppression of the track. As discussed in chapter 3, the problem of finding the best control and actuator location is the optimization problem
\begin{equation}
\begin{cases}
\min&J(u,r;x_0)\\
\text{s.t.}&\dot{x}(t)=Ax(t)+F(x(t))+B(r)u(t), \quad \forall t\in(0,T],\\
&x(0)=x_0
\end{cases}
\end{equation}

The optimality conditions use the derivative of the cost function with respect to the input and the actuator location. In that, the adjoint of the IVP needs to be calculated. The adjoint equation is the following final value problem (FVP):
\begin{equation}
\dot{p}(t)=-(A^*+F'^*_x(t))p(t)-Qx(t),\quad p(T)=0
\end{equation}

For every $x_0\in X$, the derivatives of the cost function with respect to $u$ and $r$ evaluated at $u\in \text{int}(U_{ad})$, $r\in \text{int}(K_{ad})$ are linear operators $D_u J(u,r;x_0)$ and $D_{r}J(u,r;x_0)$, respectively. Identifying each operator with an element of its underlying space, they are derived as

\begin{equation}
\begin{cases}
D_u J(u,r;x_0)&=B^*(r)p(t)+R u(t),\\
D_{r}J(u,r,x_0)&=\int_0^T(B'_{r}u(s))^*p(s)\, ds.
\end{cases}
\end{equation}





















## Optimization Algorithms
Several optimization algorithms were suggested in the literature for solving PDE-constrained optimization problems. In this section, two common optimization algorithms for solving the optimization problem will be discussed. These are projected gradient method and nonlinear conjugate gradient method. In projected gradient (or steepest descent) method, the negative of the gradient is considered as the search direction. This algorithm reads as follows:

begin{algorithm}(Projected Gradient Method)
\begin{enumerate}
\item \textbf{input} initial guesses $\ub_1\in U_{ad}$ and $\rb_1\in K_{ad}$
\item Set $n=1$
\item \textbf{while} a criteria is met \textbf{do}
\item \hspace*{0.5cm}Solve the IVP with $\ub_n$ and $\rb_n$, and find $\xb_n$
\item \hspace*{0.5cm}Solve the FVP with $\xb_n$, and find $\pb_n$
\item \hspace*{0.5cm}Evaluate $d^\ub_n\coloneqq -D_\ub J(\ub_n,\rb_n,\xb_0) $ and $d^\rb_n\coloneqq-D_{\rb}J(\ub_n,\rb_n;\xb_0)$ from \hspace*{0.5cm}(\ref{Numerics-eq-duJ}) and (\ref{Numerics-eq-drJ})
\item \hspace*{0.5cm}Obtain step lengths $s^\ub_n$ and $s^\rb_n$ by one of the line search methods discussed below  
\item \hspace*{0.5cm}Set $\ub_{n+1}=\ub_n+s_n^\ub d_n^\ub $ and $\rb_{n+1}=\rb_n+s_n^rd_n^\rb$
\item \hspace*{0.5cm}increase $n$
\item \textbf{end while}
\end{enumerate}
\end{algorithm}
