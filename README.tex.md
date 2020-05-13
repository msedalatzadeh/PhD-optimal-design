# Optimal Actuator Location for Railway Track PDE
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

### Projected Gradient Algorithm
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

Projected gradient method is typically converging to an optimizer slowly, whereas the nonlinear conjugate gradient method promises faster convergence \cite{nocedal1999}. 

### Conjugate Gradient Algorithm
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

\item \hspace*{0.5cm}\textbf{if} $\inn{d^u_{n+1}}{h^u_{n+1}}_{L^2(0,T;U)}\le 0$ \textbf{then}

\item \hspace*{1cm}Set $d^u_{n+1}=h^u_{n+1}$

\item \hspace*{0.5cm}\textbf{end if}

\item \hspace*{0.5cm}\textbf{if} $\inn{d^r_{n+1}}{h^r_{n+1}}_{K}\le 0$ \textbf{then}

\item \hspace*{1cm}Set $d^r_{n+1}=h^r_{n+1}$

\item \hspace*{0.5cm}\textbf{end if}

\item \hspace*{0.5cm}increase $n$

\item \textbf{end while}
\end{enumerate}


Several choices exist for selecting the step length $\beta^u_{n+1}$ (similarly $\beta^r_{n+1}$) of the previous algorithm. Letting $\gamma^u_{n+1}=h^u_{n+1}-h^u_n$, the following are considered for selecting the step length $\beta^u_{n+1}$ (similarly $\beta^r_{n+1}$)

\begin{flalign*}
&\text{Fletcher-Reeves:} \quad \beta^u_{n+1}=\frac{\norm{h^u_{n+1}}{U}}{\norm{h^u_n}{U}},\\
&\text{Polan-Ribi\`ere:} \quad \beta^u_{n+1}=\frac{\inn{h^u_{n+1}}{\gamma^u_{n+1}}_U}{\norm{h^u_n}{U}},\\
&\text{Hestenes-Stiefel:} \quad  \beta^u_{n+1}=\frac{\inn{h^u_{n+1}}{\gamma^u_{n+1}}_U}{\inn{d^u_n}{\gamma^u_{n+1}}_U}.
\end{flalign*}


A new formula was also proposed by Hager and Zhang; define $\bar{\beta}^u_{n+1}$ and $\eta^u_{n+1}$ as

\begin{flalign}
\bar{\beta}^u_{n+1}&=\frac{\inn{\gamma^u_{n+1}-2\frac{\normm{\gamma^u_{n+1}}_U^2}{\inn{d^u_{n}}{\gamma^u_{n+1}}_U}d^u_n}{h^u_{n+1}}_U}{\inn{d^u_{n+1}}{\gamma^u_{n+1}}_U},\notag \\
\eta^u_{n+1}&=-\frac{1}{\norm{d^u_n}{U}}\min\left\{0.01,\norm{h^u_n}{U}\right\}\notag.
\end{flalign}

Then, the formula is
\begin{equation}
\text{Hager-Zhang:}\quad \beta^u_{n+1}=\max\left\{\bar{\beta}^u_{n+1},\eta^u_{n+1}\right\}
\end{equation}

Furthermore, several schemes have been proposed to choose the step length $s^u_n$ (similarly $s^r_n$) in each iteration of previous algorithms including bisection, (strong) Wolfe conditions, Secant method.
%, and Hager-Zhang with guaranteed descent.
\begin{enumerate}
\item \textbf{Bisection \cite{buchholz2013}:} In each iteration $n$ of the algorithms, initialize $s^u_{n,1}$ and $s^r_{n,1}$. Set $m=1$. Compute $u_{n,m}=u_{n}+s^u_{n,m} d^u_{n}$, $r_{n,m}=r_{n}+s^r_{n,m} d^r_{n}$, and $J(u_{n,m},r_{n,m};x_0)$. If
\begin{equation}
J(u_{n,m},r_{n,m};x_0)\le J(u_{n},r_n;x_0),
\end{equation}
then accept the step size; otherwise, set $s^u_{n,m+1}=\frac{1}{2}s^u_{n,m}$ and $s^r_{n,m+1}=\frac{1}{2}s^r_{n,m}$ and repeat the process.
\item \textbf{Wolfe conditions \cite[Section~3.1]{nocedal1999}:} In each iteration $n$ of the algorithms, initialize $s^u_{n,1}$ and $s^r_{n,1}$. Set $m=1$. Pick constants $c_1$ and $c_2$ in the interval $(0,1)$. Compute $u_{n,m}=u_{n}+s^u_{n,m} d^u_{n}$ and $r_{n,m}=r_{n}+s^r_{n,m} d^r_{n}$ together with  $h^u_{n,m}=-D_u J(u_{n,m},r_{n,m};x_0)$ and $h^r_{n,m}=-D_r J(r_{n,m},r_{n,m};x_0)$. Iterate the step size $s^u_{n,m}$ and $s^r_{n,m}$ until the following conditions are met
\begin{subequations}\label{Numerics-eq-curvature Wolfe}
\begin{flalign}
J(u_{n,m},r_n;x_0)&\le J(u_{n},r_n;x_0)+c_1s^u_{n,m}\inn{h^u_{n,m}}{d^u_{n}}_U\\
&\quad +c_1s^r_{n,m}\inn{h^r_{n,m}}{d^r_{n}}_K\notag,\\
\inn{h^u_{n,m}}{d^u_{n}}_U&\ge c_2  \inn{h^u_{n}}{d^u_{n}}_U,\\
\inn{h^r_{n,m}}{d^r_{n}}_K&\ge c_2  \inn{h^r_{n}}{d^r_{n}}_K.
\end{flalign}
\end{subequations}
\item \textbf{Strong Wolfe conditions \cite[Section~3.1]{nocedal1999}:} Similar to Wolfe conditions except that condition (\ref{Numerics-eq-curvature Wolfe}) is replaced with
\begin{subequations}
\begin{flalign}
|\inn{h^u_{n,m+1}}{d^u_{n,m}}_U|&\le c_2  |\inn{h^u_{n,m}}{d^u_{n,m}}_U|,\\
|\inn{h^r_{n,m+1}}{d^r_{n,m}}_K|&\le c_2  |\inn{h^r_{n,m}}{d^r_{n,m}}_K|.
\end{flalign}
\end{subequations}
\item \textbf{Secant method:} The step lengths can be approximate minimizers of the function $\theta(s^u ,s^r):= J(u_n+s^u d^u_n,r_n+s^rd^r_n;x_0)$. For instance, letting $\sigma^u $ and $\sigma^r$ be some positive constants, an approximate minimizer of $\theta(s^u ,s^r)$ can be derived by using secant formula as
\begin{equation}
s^u =\frac{\theta_{s^u }(0,0)}{\theta_{s^u }(\sigma^u ,0)-\theta_{s^u }(0,0)},\; s^r=\frac{\theta_{s^r}(0,0)}{\theta_{s^r}(0,\sigma^r)-\theta_{s^r}(0,0)},
\end{equation}
where the subscripts indicate partial derivatives. In the first iteration, the constants $\sigma^u $ and $\sigma^r$ are chosen arbitrary; in next iterations, they are set to the values of $s^u $ and $s^r$ found in the previous iteration \cite{herzog2010}. Accordingly, from the definition of $\theta(s^u ,s^r)$, and by arbitrary initializing $s^u_0$ and $s^r_0$, it follow that 
\begin{subequations}
\begin{flalign}\label{Numerics-eq-secant}
s^u_n &=-\frac{\inn{h^u_n}{d^u_n}_U}{\inn{h^u_n+D_u J(u_n+s^u_{n-1} d^u_n,r_n;x_0)}{d^u_n}_U},\\ 
s^r_n &=-\frac{\inn{h^r_n}{d^r_n}_K}{\inn{h^r_n+D_r J(u_n,r_n+s^r_{n-1} d^r_n;x_0)}{d^u_n}_K}.
\end{flalign}
\end{subequations} 
%\item \textbf{Hager-Zhang with guaranteed descent}:
\end{enumerate}