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
J(u_{n,m},r_n;x_0)&\le J(u_{n},r_n;x_0)+c_1s^u_{n,m}\langle h^u_{n,m} , d^u_{n} \rangle_U\\
&\quad +c_1s^r_{n,m}\langle h^r_{n,m} , d^r_{n}\rangle_K\notag,\\
\langle h^u_{n,m} , d^u_{n} \rangle_U&\ge c_2  \langle h^u_{n}}{d^u_{n} \rangle_U,\\
\langle h^r_{n,m} , d^r_{n} \rangle_K&\ge c_2  \langle h^r_{n}}{d^r_{n} \rangle_K.
\end{flalign}
\end{subequations}

\item \textbf{Strong Wolfe conditions \cite[Section~3.1]{nocedal1999}:} Similar to Wolfe conditions except that condition (\ref{Numerics-eq-curvature Wolfe}) is replaced with
\begin{subequations}
\begin{flalign}
|\langle h^u_{n,m+1} , d^u_{n,m} \rangle_U|&\le c_2  |\langle h^u_{n,m} , d^u_{n,m} \rangle_U|,\\
|\langle h^r_{n,m+1} , d^r_{n,m} \rangle_K|&\le c_2  |\langle h^r_{n,m} , d^r_{n,m} \rangle_K|.
\end{flalign}
\end{subequations}

\item \textbf{Secant method:} The step lengths can be approximate minimizers of the function $\theta(s^u ,s^r):= J(u_n+s^u d^u_n,r_n+s^rd^r_n;x_0)$. For instance, letting $\sigma^u $ and $\sigma^r$ be some positive constants, an approximate minimizer of $\theta(s^u ,s^r)$ can be derived by using secant formula as

\begin{equation}
s^u =\frac{\theta_{s^u }(0,0)}{\theta_{s^u }(\sigma^u ,0)-\theta_{s^u }(0,0)},\; s^r=\frac{\theta_{s^r}(0,0)}{\theta_{s^r}(0,\sigma^r)-\theta_{s^r}(0,0)},
\end{equation}

where the subscripts indicate partial derivatives. In the first iteration, the constants $\sigma^u $ and $\sigma^r$ are chosen arbitrary; in next iterations, they are set to the values of $s^u $ and $s^r$ found in the previous iteration \cite{herzog2010}. Accordingly, from the definition of $\theta(s^u ,s^r)$, and by arbitrary initializing $s^u_0$ and $s^r_0$, it follow that 

\begin{subequations}
\begin{flalign}\label{Numerics-eq-secant}
s^u_n &=-\frac{\langle h^u_n, d^u_n \rangle_U}{\langle h^u_n+D_u J(u_n+s^u_{n-1} d^u_n,r_n;x_0) , d^u_n \rangle_U},\\ 
s^r_n &=-\frac{\langle h^r_n , d^r_n \rangle_K}{\langle h^r_n+D_r J(u_n,r_n+s^r_{n-1} d^r_n;x_0) , d^u_n \rangle_K}.
\end{flalign}
\end{subequations} 
%\item \textbf{Hager-Zhang with guaranteed descent}:
\end{enumerate}



## Approximation Scheme
In each iteration, the IVP and FVP are solved numerically. To find a numerical solution to the IVP and FVP, a finite-dimensional approximation is needed. Let $n_z$, $n_u$, and $n_r$ indicate the dimension of finite-dimensional subspaces of $X$, $U$, and $K $, respectively.
To avoid multiple subscripts, let $n=[n_z,n_u,n_r]$, and denote the subspaces by $X_n$, $U_n$, and $K_n$. 
%a set of bases for the underlying spaces is needed. 
%Consider finite-dimensional approximation 
%Consider only a finite number of the bases, let $n_{z}$, $n_{u}$, and $n_r$ be natural numbers, and use $n=[n_z,n_u,n_r]$ to indicate a finite-dimensional component.  
%use $i=1,...,n_z$, and $j=1,...,n_u$, $k=1,...,n_r$ to enumerate the bases. Let $\bm{e}^X_i$, $\bm{e}^U_j$, and $\bm{e}^K_k$ denote the orthonormal bases of $X$, $U$, and $K$, respectively. Let $X_{n_z}$, $U_{n_u}$, and $K_{n_r}$ be subspaces spanned by all $\bm{e}^X_i$, $\bm{e}^U_j$, and $\bm{e}^K_k$, respectively. 
Also, let $\proj_{z}:X\to X_{n}$, $\proj_{u}:U\toU_{n}$, and $\proj_{r}:K \to K_{n}$ be the projection of $X$, $U $, and $K $ onto $X_{n}$, $U_{n}$, and $K_{n}$, respectively. Define sets $K_{ad,n}$ and $U_{ad,n}$ in a similar way. For every $r\in K_{ad,n}$, consider the finite-dimensional linear operators $\A_n\in \mc{L}(X_{n})$ and $\B_n(r) \in \mc{L}(U_{n},X_{n})$, and $\mc Q_n:= \mc Q|_{X_{n}}$.

There are different techniques to handle the nonlinear operators  $\F(\cdot)$, $\F'_{\cdot}$, $\B(\cdot)$, and $\B'_{\cdot}$. A common way is to approximate the nonlinear operator $\F(\cdot)$ with an operator $\F_n(\cdot)$ that coincides with $\F(\cdot)$ on $X_{n}$. The operators $\F'_{\cdot}$, $\B(\cdot)$, and $\B'_{\cdot}$ can be approximated in similar ways. 

Then, the approximated IVP and FVP are governed by
\begin{equation}\label{Numerics-eq-truncated}
\begin{array}{ll}
\dot{x}(t)=\A_nx(t)+\F_n(x(t))+\B_n(r)u(t), & x(0)=x_{n0}:=\proj_{n}x_0,\\[2mm]
\dot{\pb}(t)=-(\A_n^*+\F'^*_{n,x(t)})\pb(t)-\mc Q_nx(t), & \pb(T)=0.
\end{array}
\end{equation}
For the optimality conditions, the operator $(\mc{B}'_{n,r}u)^*:X_{n}\to K_{n}$ is defined by a sesquilinear form
\begin{equation}\label{Numerics-eq-sesquilinear}
\langle (\B'_{n,r}u)^*\pb}{r}_K=\langle \pb}{(\B'_{n,r}r)u}, \quad \forall (u,\pb,r)\in  U_{n}\times X_{n}\times K_{n}.
\end{equation}
Then, letting $\mc R_n:= \mc R|_{X_{n}}$, the approximated optimality conditions are 
\begin{flalign}
D_u J_n(u,r;x_{n0})&=\B_n^*(r)\pb(t)+\mc{R}_n u(t),\label{Numerics-eq-duJ}\\
D_{r}J_n(u,r,x_{n0})&=\int_0^T(\B'_{n,r}u(s))^*\pb(s)\, ds.\label{Numerics-eq-drJ}
\end{flalign}
These operators should satisfy a set of assumption to be qualified as an approximation of the operators in the original system. Assumptions A1-A3 in \cite{morris2011linear} ensures this for a linear system with infinite horizon cost function.

By means of a basis for the underlying spaces, the approximation can be fully realized. This yields an approximation that satisfies assumptions A1-A3 in \cite{morris2011linear}. Using $i\in \mathbb{N}_n$ to enumerate the bases, let $\bm{e}^X_i$, $\bm{e}^U_i$, and $\bm{e}^K_i$ denote orthonormal bases of $X$, $U$, and $K$, respectively. It is assumed that $\bm{e}^X_i\in D(\A)$ for all $i\in \mathbb{N}_n$. Denoted by $x_i$ and $p_i$ are projections of the state $x$ and adjoint state $\pb$ onto the one-dimensional subspaces spanned by $\bm{e}^X_i$. Define $u_i$ and $r_i$ in a similar way. Consider the vectors $\xv$, $\uv$, $\rv$, and $ F(\xv)$ as
\begin{equation}
\xv:=\begin{bmatrix}
x_1\\
\vdots\\
x_{n}
\end{bmatrix}, \quad \pv:=\begin{bmatrix}
p_1\\
\vdots\\
p_n
\end{bmatrix}, \quad \uv:= \begin{bmatrix}
u_1\\
\vdots\\
u_n
\end{bmatrix}, \quad \rv:= \begin{bmatrix}
r_1\\
\vdots\\
r_n
\end{bmatrix}
\end{equation}
\begin{equation}
\xv_0:=\begin{bmatrix}
\langle x_0}{\bm{e}^X_1}\\
\vdots\\
\langle x_0}{\bm{e}^X_{n}}
\end{bmatrix}, \quad  F(\xv):=\begin{bmatrix}
\langle \F(x)}{\bm{e}^X_1}\\
\vdots\\
\langle \F(x)}{\bm{e}^X_{n}}
\end{bmatrix},
\end{equation}
and matrices $ A_n$, $ Q_n$, and ${dF}_n({\xv})$ with $i$th row and $j$th column, $i,j=1,..., {n_z}$, as
\begin{equation}
{A}_{ij}:=\langle \A \bm{e}^X_j}{\bm{e}^X_i}, \quad {Q}_{ij}:=\langle \mc Q \bm{e}^X_j}{\bm{e}^X_i}, \quad {dF}_{ij}(\xv):=\langle \F'_{x}\bm{e}^X_j}{\bm{e}^X_i},
\end{equation}
matrix $ B(\rv)$ with $i=1,...,{n_z}$ and $j=1,...,{n_u}$ as
\begin{equation}
B_{ij}(\rv):=\langle \B(r)\bm{e}^U_j}{\bm{e}^X_i} .
\end{equation}
The superscript $^*$ will denote conjugate transpose, $ A^*=\bar{ A}^T$. A finite-dimensional state-space representation of the approximated IVP and FVP is
\begin{equation}\label{Numerics-eq-approximate IVP FVP}
\begin{array}{ll}
\dot{\xv}(t)= A_n\xv(t)+ F_n(\xv(t))+ B_n(\rv)\uv(t), & \xv(0)=\xv_0,\\[2mm]
\dot{\pv}(t)=-( A_n^*+{dF}^*_n({\xv(t)}))\pv(t)- Q_n\xv(t), & \pv(T)=0.
\end{array}
\end{equation}

Also, define the matrix $ R_n$ with $i,j=1,...,{n_u}$ as
\begin{equation}
{R}_{ij}:=\langle \mc R \bm{e}^U_j}{\bm{e}^U_i},
\end{equation}
The optimality condition (\ref{Numerics-eq-duJ}) becomes 
\begin{equation}\label{Numerics-eq-aproximate DuJ}
D_u J_n(\uv,\rv;\xv_0)= B^*(\rv)\pv(t)+ R\uv(t).
\end{equation}
To write the optimality condition (\ref{Numerics-eq-drJ}) in a vector form, use the sesquilinear form (\ref{Numerics-eq-sesquilinear}) together with Corollary 5.8 in \cite{edalatzadehSICON}, let $i=1,...,n_u$, $j=1,...,{n_r}$, and $k=1,...,{n_z}$, define the array ${dB}_{n}(\rv)$ as
\begin{equation}
{dB}_{ijk}(\rv)=\langle b'_{i,r}e_j^\mathbb{K}}{e_k^\mathbb{Z}},
\end{equation}
see \cite{edalatzadehSICON} for the definition of $b'_{j,r}$. 
This optimality condition becomes
\begin{flalign}\label{Numerics-eq-approximate DrJ}
D_r J_{n,j}(\uv,\rv,\xv_0)&=\int_0^T\sum_{i=1}^{n_u}\sum_{k=1}^{n_z}\bar{u}_i(s){dB}_{ijk}(\rv)p_k(s)\, ds.
\end{flalign}
Knowing the components of the above optimality condition, this equation is compactly written as
\begin{flalign}
D_r J_n(\uv,\rv,\xv_0)&=\int_0^T\uv^*(s){dB}_n({\rv})\pv(s)\, ds.
\end{flalign}

%The time derivatives can be approximated using Euler method. Consider a partition $0=t_0<t_1<\cdots<t_{N}=T$, and let $\tau_i=t_i-t_{i-1}$, $i=1,2,...,N$. The time derivatives $\dot{\xv}(t)$ and $\dot{\pv}(t)$ are approximated as
%\begin{subequations}
%\begin{flalign}
%\dot{\xv}(t_i)&=\frac{1}{\tau_i}\left(\xv(t_i)-\xv(t_{i-1})\right)\\
%\dot{\pv}(t_i)&=\frac{1}{\tau_i}\left(\pv(t_i)-\pv(t_{i-1})\right)
%\end{flalign}
%\end{subequations}
%Based on the time step at which the right hand side of the IVP and FVP are calculated, various schemes appear. The common ones are
%\begin{enumerate}
%\item \textbf{Explicit scheme:} The right hand side of IVP and FVP will be calculated at a the previous time step (i.e. $t=t_{i-1}$).
%\begin{subequations}
%\end{subequations}
%\item \textbf{Implicit scheme:} The right hand side of IVP and FVP will be calculated at a the current time step (i.e. $t=t_{i}$).
%\item \textbf{Semi-implicit scheme:} The nonlinear parts of the IVP and FVP are calculated at a the previous time step, whereas the linear parts are calculated at the current step time.
%\end{enumerate}



## Simulation Results
A basis is chosen for the numerical simulation. Let 
\begin{equation}
c_i=\sqrt{\frac{2\ell^3\pi^4 i^4}{EI\pi^4 i^4+k\ell^4+\rho a \ell^4 \pi^4 i^4}}.
\end{equation}
It is straightforward to show that the following sequence forms an orthonormal basis for $X$:
\begin{subequations}\label{Numerics-basis}
\begin{flalign}
\bm{e}^X_{2i-1}&=\left(\frac{c_i}{\pi^2i^2}\sin(\frac{\pi i}{\ell}\xi),c_i\sin(\frac{\pi i}{\ell}\xi) \right),\quad i\in \mathbb{N},\\ 
\bm{e}^X_{2i}&=\left(-\frac{c_i}{\pi^2i^2}\sin(\frac{\pi i}{\ell}\xi),c_i\sin(\frac{\pi i}{\ell}\xi) \right),\quad i\in \mathbb{N}.
\end{flalign}
\end{subequations}
Set $\mc{Q}(w,v)=(c_w w,c_v v)$ and $\mc{R}=c_u$ in the cost function for some positive constants $c_w$, $c_v$, and $c_u$. Then, the FVP and the optimality conditions (\ref{Numerics-eq-duJ}) and (\ref{Numerics-eq-drJ}) follow immediately. Letting $i=1,...,2n$, and
\begin{equation}\notag
\begin{array}{ll}
a^1_i=-\frac{\mu l^4+C_d\pi ^4i^4}{2l^3}c_i^2, &a^2_i=\frac{kl^4+EI\pi ^4i^4}{l^3i^2\pi ^2}c_i^2\\
q^1_i=c_i(c_v-c_w)\rho a \ell ^4 n^4\pi ^4+c_w/c_i^2,& q^2_i=c_i(c_v+c_w)\rho a \ell ^4 n^4\pi ^4-c_w/c_i^2,
\end{array}
\end{equation}
the matrices $ A_n$, $ Q_n$, and $ B_n$ are calculated as
\begin{flalign*}
 A_n=&\text{blockdiag}\begin{bmatrix}
a^1_i& a^1_i+a^2_i\\
a^1_i-a^2_i & a^1_i
\end{bmatrix},\\
 Q_n=& \text{blockdiag} \begin{bmatrix}
q^1_i& q^2_i\\
q^2_i& q^1_i
\end{bmatrix},
\end{flalign*}

\begin{equation*}
B_{(2i-1)1}=B_{(2i)1}={c_i}\int_0^\ell b(\xi;r)\sin(\frac{\pi i}{\ell}\xi) \, d\xi.
\end{equation*}
The components of the nonlinear operator $F(\cdot)$ are calculated as
\begin{flalign}
&F_{(2i-1)1}=F_{(2i)1}\notag\\
&\quad=-\alpha c_i\int_0^\ell \left(\sum_{j=1}^{n}\frac{c_j(x_{2j-1}-x_{2j})}{\pi^2 j^2}\sin(\frac{\pi j}{\ell}\xi)\right)^3 \sin(\frac{\pi i}{\ell}\xi) \,d\xi\\
&dF_{(2i)(2j)}=dF_{(2i-1)(2j)}=dF_{(2i)(2j-1)}=dF_{(2i-1)(2j-1)}\notag\\
&\quad =-3\alpha c_jc_i\int_0^\ell \left(\sum_{k=1}^{n}\frac{x_kc_k}{\pi^2 k^2}\sin(\frac{\pi k}{\ell}\xi)\right)^2\sin(\frac{\pi j}{\ell}\xi)\sin(\frac{\pi i}{\ell}\xi) \, d\xi
\end{flalign}
For the derivative of the input operator with respect to the actuator location, let $b_r(\xi;r)$ denote the derivative of $b(\xi;r)$ with respect to $r$, then
\begin{equation}
{dB}_{n}=\text{row}\left({c_i}\int_0^\ell b_r(\xi;r)\sin(\frac{\pi i}{\ell}\xi) \, d\xi\right).
\end{equation}
The function $b(\xi;r)$ is chosen as follows:
\begin{equation}
b(\xi;r)=\begin{cases}
(\xi-r+\delta)^2(\xi-r-\delta)^2, \quad |\xi-r|\le \delta\\
0\quad \text{otherwise}
\end{cases}
\end{equation}
Also, it follows that $ R_n=c_u$. Now, the approximate IVP and FVP is governed by (\ref{Numerics-eq-approximate IVP FVP}); the optimality conditions are given by (\ref{Numerics-eq-aproximate DuJ}) and (\ref{Numerics-eq-approximate DrJ}). 

In the railway track model, if the variables $w$, $t$, and $\xi$ are appropriately substituted with dimensionless variables, a dimensionless PDE model for the railway track can be derived. Let $c_1$, $c_2$, $c_3$, and $c_4$ be some generic constants, a dimensionless railway track is formatted as   
\begin{equation}
\begin{cases}
\frac{\partial^2 w}{\partial t^2}+\frac{\partial}{\partial \xi^2}(\frac{\partial^2 w}{\partial \xi^2}+c_1\frac{\partial ^3 w}{\partial \xi^2 \partial t})+c_2 \frac{\partial w}{\partial t}+c_3w +c_4 w^3=u(\xi,t),\\[1mm]
\allowdisplaybreaks w(\xi,0)=w_0(\xi), \quad \frac{\partial w}{\partial t}(\xi,0)=v_0(\xi),\\[1mm]
\allowdisplaybreaks w(0,t)=w(1,t)=0,\\[1mm]
\allowdisplaybreaks \frac{\partial ^2w}{\partial \xi^2}(0,t)+c_1\frac{\partial ^3w}{\partial \xi^2\partial t}(0,t)=0,\\[1mm]
\frac{\partial^2 w}{\partial \xi^2}(1,t)+c_1\frac{\partial ^3w}{\partial \xi^2\partial t}(1,t)=0. 
\end{cases}
\end{equation}
The relative value of the coefficients is important. The coefficients are chosen with nominal values 
\begin{equation}
c_1=10^{-4}, \quad c_2=0.1, \quad c_3=1, \quad c_4=10.
\end{equation}
The coefficients $c_1$, $c_2$ and $c_4$ will be changed in simulations to observe their influence. Moreover, the final interval time is set to $\tau=10$. This gives the state of the controlled system enough time to settle. In addition, we choose $\delta=5\times 10^{-5}$ so that the input force is concentrated on a relatively small region on the track. In the cost function, we choose the same weights for the deflection, rate of deflection, and input; so $c_w=c_v=c_u=1$ is selected for the simulations. 

Given an order of approximation, the initial conditions are chosen such that all modes are excited. The initial conditions are chosen from
\begin{equation}
x_0=\left(2,\, 3,\, \frac{2}{2},\, \frac{3}{2},\, \frac{2}{4},\, \frac{3}{4},\, \frac{2}{8},\, \frac{3}{8},\, \frac{2}{16},\, \frac{3}{16}\right).
\end{equation}
The order of approximation is equal to the dimension of an initial condition. For example, if the order of approximation is 4, the initial condition is
\begin{equation}
x_0=\left(2,\, 3,\, \frac{2}{2},\, \frac{3}{2}\right).
\end{equation}
The initial condition is illustrated in \Cref{Numerics-incond} for the 10th order approximation. 
\begin{figure}[h]
\centering
\includegraphics[width=0.4\linewidth]{"MATLAB- Railway track - new codes/initial condition"}
\captionsetup{font=small, width=0.7\linewidth}
\caption{Graph of the the initial condition for the simulations.}
\label{Numerics-incond}
\end{figure}

Simulations were conducted using the software MATLAB. The ODE solver \texttt{ode15s} was used to solve the finite-dimensional approximation of the system. MATLAB optimization routine \texttt{fmincon} was also used as the optimization algorithm. The convergence of the approximation method is illustrated in \Cref{fig-order damped,fig-order undamped}. It is observed that beyond 6th order approximation, increasing the approximation order will not make a noticeable difference. \Cref{fig-nonlin damped,fig-nonlin undamped} compare the cost and optimal input for the linear and nonlinear model in the presence and absence of damping. These figures indicate a significant change in the cost of control and in the optimal input. \Cref{fig-nonlin cost} shows how the cost and optimal location of actuators change when the coefficient of nonlinearity, $\alpha$, is increased. As a general rule of thumb, increasing $\alpha$ increases the cost of control. Moreover, \Cref{fig-visc,fig-kelvin} show how the cost and location of actuators change when the coefficient of viscous and Kelvin-Voigt damping are decreased. Simulations show that the optimal location of actuators moves away from the center as the damping is decreased. Also, an interesting observation is made in \Cref{fig-kelvin} where local optimizers appear by decreasing the coefficient of Kelvin-Voigt damping. Lastly, \Cref{fig-center} shows the improvement in the performance of the control system when the optimal location is chosen for the actuator over the center location.  

\begin{figure}[h]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/increasing order"}
\captionsetup{font=small, width=\linewidth}
\caption{Convergence of the numerical scheme for different orders of approximation in undamped beam. No significant improvement is observed for 4th order approximation or higher.}
\label{fig-order damped}
\end{figure}


\begin{figure}[H]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/increasing order -undamped"}
\captionsetup{font=small, width=\linewidth}
\caption{Convergence of the numerical scheme for different orders of approximation in damped beam. No significant improvement is observed for 6th order approximation or higher.}
\label{fig-order undamped}
\end{figure}


\begin{figure}[H]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/linearVsnonlinear-damping"}
\captionsetup{font=small, width=\linewidth}
\caption{Comparison of the optimal input and cost function in linear and nonlinear damped beam. The cost of control increases by increasing the nonlinearity.}
\label{fig-nonlin damped}
\end{figure}



\begin{figure}[H]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/linearVSnonlinear-nodamping"}
\captionsetup{font=small, width=\linewidth}
\caption{Comparison of the optimal input and cost function in linear and nonlinear undamped beam. The cost of control increases by increasing the nonlinearity.}
\label{fig-nonlin undamped}
\end{figure}


\newpage
\begin{figure}[h]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/alpha"}
\captionsetup{font=small, width=\linewidth}
\caption{Effect of nonlinearity on the cost function. The optimal actuator locations do not change significantly despite the change in the cost.}
\label{fig-nonlin cost}
\end{figure}





\begin{figure}[H]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/mu"}
\captionsetup{font=small, width=\linewidth}
\caption{Effect of viscous damping on the cost function. The optimal actuator locations move away from center as the damping is decreased.}
\label{fig-visc}
\end{figure}





\begin{figure}[H]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/Cd"}
\captionsetup{font=small, width=\linewidth}
\caption{Effect of Kelvin-Voigt damping on the cost function. If $C_d=0$, the beam models is hyperbolic. The optimal actuator locations move away from center as the damping is decreased.}
\label{fig-kelvin}
\end{figure}




\begin{figure}[H]
\centering
\includegraphics[width=0.6\linewidth]{"C:/Users/sajjad/Dropbox/Edalatzadeh/Numerics on railway model/MATLAB- Railway track - new codes/optimalVSmidpoint"}
\captionsetup{font=small, width=\linewidth}
\caption{ Comparison of optimal inputs: optimal location vs center. Actuators on optimal locations improve the control input.}
\label{fig-center}
\end{figure}







## References