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

Projected gradient method is typically converging to an optimizer slowly, whereas the nonlinear conjugate gradient method promises faster convergence \cite{nocedal1999}. The nonlinear conjugate gradient method reads as follows:
\begin{alg}(Nonlinear Conjugate Gradient Method)
\begin{enumerate}
\item \textbf{input} initial guesses $\ub_1\in U_{ad}$ and $\rb_1\in K_{ad}$

\item Set $n=1$

\item Set $d^\ub_n= h^\ub_n\coloneqq -D_\ub J(\ub_n,\rb_n,\xb_0)$ and $d^\rb_n= h^\rb_n\coloneqq-D_{\rb}J(\ub_n,\rb_n;\xb_0)$

\item \textbf{while} a criteria is met \textbf{do}

\item \hspace*{0.5cm}Solve the IVP for $\ub_n$ and $\rb_n$, and find $\xb_n$

\item \hspace*{0.5cm}Solve the FVP for $\xb_n$, and find $\pb_n$

\item \hspace*{0.5cm}Obtain step lengths $s^\ub_n$ and $s^\rb_n$ using, e.g., secant method in (\ref{Numerics-eq-secant})

\item \hspace*{0.5cm}Set $\ub_{n+1}=\ub_n+s_n^\ub d_n^\ub $ and $\rb_{n+1}=\rb_n+s_n^\rb d_n^\rb$

\item \hspace*{0.5cm}Solve the IVP for $\ub_{n+1}$ and $\rb_{n+1}$, and find $\xb_{n+1}$

\item \hspace*{0.5cm}Solve the FVP for $\xb_{n+1}$, and find $\pb_{n+1}$

\item \hspace*{0.5cm}Evaluate $h^\ub_{n+1}\coloneqq -D_\ub J(\ub_{n+1},\rb_{n+1},\xb_0) $ and $h^\rb_{n+1}\coloneqq-D_{\rb}J(\ub_{n+1},\rb_{n+1};\xb_0)$ \hspace*{0.5cm}from (\ref{Numerics-eq-duJ}) and (\ref{Numerics-eq-drJ})

\item \hspace*{0.5cm}Determine step lengths $\beta^\ub_{n+1}$ and $\beta^\rb_{n+1}$ using, e.g., Fletcher-Reeves \hspace*{0.5cm}or Polak-Ribi\`ere formula \cite[Section~5.2]{nocedal1999}

\item \hspace*{0.5cm}Set $d^\ub_{n+1}\coloneqq h^\ub_{n+1}+\beta^\ub_{n+1}d^\ub_{n}$ and $d^\rb_{n+1}\coloneqq h^\rb_{n+1}+\beta^\rb_{n+1}d^\rb_{n}$

\item \hspace*{0.5cm}\textbf{if} $\inn{d^\ub_{n+1}}{h^\ub_{n+1}}_{L^2(0,T;\cs)}\le 0$ \textbf{then}

\item \hspace*{1cm}Set $d^\ub_{n+1}=h^\ub_{n+1}$

\item \hspace*{0.5cm}\textbf{end if}

\item \hspace*{0.5cm}\textbf{if} $\inn{d^\rb_{n+1}}{h^\rb_{n+1}}_{\as}\le 0$ \textbf{then}

\item \hspace*{1cm}Set $d^\rb_{n+1}=h^\rb_{n+1}$

\item \hspace*{0.5cm}\textbf{end if}

\item \hspace*{0.5cm}increase $n$

\item \textbf{end while}
\end{enumerate}
\end{alg}

Several choices exist for selecting the step length $\beta^\ub_{n+1}$ (similarly $\beta^\rb_{n+1}$) of the previous algorithm \cite{hager2006survey}. Letting $\gamma^\ub_{n+1}=h^\ub_{n+1}-h^\ub_n$, the following are for selecting the step length $\beta^\ub_{n+1}$ (similarly $\beta^\rb_{n+1}$) considered in this paper
\begin{subequations}
\begin{flalign}
&\text{Fletcher-Reeves:} \quad \beta^\ub_{n+1}=\frac{\norm{h^\ub_{n+1}}{\cs}}{\norm{h^\ub_n}{\cs}},\\
&\text{Polan-Ribi\`ere:} \quad \beta^\ub_{n+1}=\frac{\inn{h^\ub_{n+1}}{\gamma^\ub_{n+1}}_\cs}{\norm{h^\ub_n}{\cs}},\\
&\text{Hestenes-Stiefel:} \quad  \beta^\ub_{n+1}=\frac{\inn{h^\ub_{n+1}}{\gamma^\ub_{n+1}}_\cs}{\inn{d^\ub_n}{\gamma^\ub_{n+1}}_\cs}.
\end{flalign}
\end{subequations}
A new formula was also proposed by Hager and Zhang \cite{hager2005new}. Define $\bar{\beta}^\ub_{n+1}$ and $\eta^\ub_{n+1}$ as
\begin{flalign}
\bar{\beta}^\ub_{n+1}&=\frac{\inn{\gamma^\ub_{n+1}-2\frac{\normm{\gamma^\ub_{n+1}}_\cs^2}{\inn{d^\ub_{n}}{\gamma^\ub_{n+1}}_\cs}d^\ub_n}{h^\ub_{n+1}}_\cs}{\inn{d^\ub_{n+1}}{\gamma^\ub_{n+1}}_\cs},\notag \\
\eta^\ub_{n+1}&=-\frac{1}{\norm{d^\ub_n}{\cs}}\min\left\{0.01,\norm{h^\ub_n}{\cs}\right\}\notag.
\end{flalign}
Then, the formula is
\begin{equation}
\text{Hager-Zhang:}\quad \beta^\ub_{n+1}=\max\left\{\bar{\beta}^\ub_{n+1},\eta^\ub_{n+1}\right\}
\end{equation}

Furthermore, several schemes have been proposed to choose the step length $s^\ub_n$ (similarly $s^\rb_n$) in each iteration of previous algorithms including bisection, (strong) Wolfe conditions, Secant method.
%, and Hager-Zhang with guaranteed descent.
\begin{enumerate}
\item \textbf{Bisection \cite{buchholz2013}:} In each iteration $n$ of the algorithms, initialize $s^\ub_{n,1}$ and $s^\rb_{n,1}$. Set $m=1$. Compute $\ub_{n,m}=\ub_{n}+s^\ub_{n,m} d^\ub_{n}$, $\rb_{n,m}=\rb_{n}+s^\rb_{n,m} d^\rb_{n}$, and $J(\ub_{n,m},\rb_{n,m};\xb_0)$. If
\begin{equation}
J(\ub_{n,m},\rb_{n,m};\xb_0)\le J(\ub_{n},\rb_n;\xb_0),
\end{equation}
then accept the step size; otherwise, set $s^\ub_{n,m+1}=\frac{1}{2}s^\ub_{n,m}$ and $s^\rb_{n,m+1}=\frac{1}{2}s^\rb_{n,m}$ and repeat the process.
\item \textbf{Wolfe conditions \cite[Section~3.1]{nocedal1999}:} In each iteration $n$ of the algorithms, initialize $s^\ub_{n,1}$ and $s^\rb_{n,1}$. Set $m=1$. Pick constants $c_1$ and $c_2$ in the interval $(0,1)$. Compute $\ub_{n,m}=\ub_{n}+s^\ub_{n,m} d^\ub_{n}$ and $\rb_{n,m}=\rb_{n}+s^\rb_{n,m} d^\rb_{n}$ together with  $h^\ub_{n,m}=-D_\ub J(\ub_{n,m},\rb_{n,m};\xb_0)$ and $h^\rb_{n,m}=-D_\rb J(\rb_{n,m},\rb_{n,m};\xb_0)$. Iterate the step size $s^\ub_{n,m}$ and $s^\rb_{n,m}$ until the following conditions are met
\begin{subequations}\label{Numerics-eq-curvature Wolfe}
\begin{flalign}
J(\ub_{n,m},\rb_n;\xb_0)&\le J(\ub_{n},\rb_n;\xb_0)+c_1s^\ub_{n,m}\inn{h^\ub_{n,m}}{d^\ub_{n}}_\cs\\
&\quad +c_1s^\rb_{n,m}\inn{h^\rb_{n,m}}{d^\rb_{n}}_\as\notag,\\
\inn{h^\ub_{n,m}}{d^\ub_{n}}_\cs&\ge c_2  \inn{h^\ub_{n}}{d^\ub_{n}}_\cs,\\
\inn{h^\rb_{n,m}}{d^\rb_{n}}_\as&\ge c_2  \inn{h^\rb_{n}}{d^\rb_{n}}_\as.
\end{flalign}
\end{subequations}
\item \textbf{Strong Wolfe conditions \cite[Section~3.1]{nocedal1999}:} Similar to Wolfe conditions except that condition (\ref{Numerics-eq-curvature Wolfe}) is replaced with
\begin{subequations}
\begin{flalign}
|\inn{h^\ub_{n,m+1}}{d^\ub_{n,m}}_\cs|&\le c_2  |\inn{h^\ub_{n,m}}{d^\ub_{n,m}}_\cs|,\\
|\inn{h^\rb_{n,m+1}}{d^\rb_{n,m}}_\as|&\le c_2  |\inn{h^\rb_{n,m}}{d^\rb_{n,m}}_\as|.
\end{flalign}
\end{subequations}
\item \textbf{Secant method:} The step lengths can be approximate minimizers of the function $\theta(s^\ub ,s^\rb)\coloneqq J(\ub_n+s^\ub d^\ub_n,\rb_n+s^rd^\rb_n;\xb_0)$. For instance, letting $\sigma^\ub $ and $\sigma^\rb$ be some positive constants, an approximate minimizer of $\theta(s^\ub ,s^\rb)$ can be derived by using secant formula as
\begin{equation}
s^\ub =\frac{\theta_{s^\ub }(0,0)}{\theta_{s^\ub }(\sigma^\ub ,0)-\theta_{s^\ub }(0,0)},\; s^\rb=\frac{\theta_{s^\rb}(0,0)}{\theta_{s^\rb}(0,\sigma^\rb)-\theta_{s^\rb}(0,0)},
\end{equation}
where the subscripts indicate partial derivatives. In the first iteration, the constants $\sigma^\ub $ and $\sigma^\rb$ are chosen arbitrary; in next iterations, they are set to the values of $s^\ub $ and $s^\rb$ found in the previous iteration \cite{herzog2010}. Accordingly, from the definition of $\theta(s^\ub ,s^\rb)$, and by arbitrary initializing $s^\ub_0$ and $s^\rb_0$, it follow that 
\begin{subequations}
\begin{flalign}\label{Numerics-eq-secant}
s^\ub_n &=-\frac{\inn{h^\ub_n}{d^\ub_n}_\cs}{\inn{h^\ub_n+D_\ub J(\ub_n+s^\ub_{n-1} d^\ub_n,\rb_n;\xb_0)}{d^\ub_n}_\cs},\\ 
s^\rb_n &=-\frac{\inn{h^\rb_n}{d^\rb_n}_\as}{\inn{h^\rb_n+D_\rb J(\ub_n,\rb_n+s^\rb_{n-1} d^\rb_n;\xb_0)}{d^\ub_n}_\as}.
\end{flalign}
\end{subequations} 
%\item \textbf{Hager-Zhang with guaranteed descent}:
\end{enumerate}