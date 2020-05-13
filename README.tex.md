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
\left\{ \begin{array}{ll}
\min&J(\ub,\rb;\xb_0)\\
\text{s.t.}&\dot{\xb}(t)=\mc{A}\xb(t)+\F(\xb(t))+\mc{B}(\rb)\ub(t), \quad \forall t\in(0,T],\\
&\xb(0)=\xb_0
\end{array} \right. \tag{P} \label{Numerics-eq-optimal problem}
\end{equation}
The fitst order optimality conditions were derived in chapter 3. The optimality conditions use the derivative of the cost function with respect to the input and the actuator location. In that, the adjoint of the IVP needs to be calculated. The adjoint equation is the following final value problem (FVP):
\begin{equation}\label{Numerics-eq-FVP}
\dot{\pb}(t)=-(\mc{A}^*+\F'^*_{\xb(t)})\pb(t)-\mc{Q}\xb(t),\quad \pb(T)=0 \tag{FVP}
\end{equation}
For every $\xb_0\in \ss$, the derivatives of the cost function with respect to $\ub$ and $\rb$ evaluated at $\ub\in \text{int}(U_{ad})$, $\rb\in \text{int}(K_{ad})$ are linear operators $D_\ub J(\ub,\rb;\xb_0)$ and $D_{\rb}J(\ub,\rb;\xb_0)$, respectively. Identifying each operator with an element of its underlying space, they are derived as
\begin{subequations}
\begin{flalign}
D_\ub J(\ub,\rb;\xb_0)&=\B^*(\rb)\pb(t)+\mc{R} \ub(t),\label{Numerics-eq-duJ}\\
D_{\rb}J(\ub,\rb,\xb_0)&=\int_0^T(\B'_{\rb}\ub(s))^*\pb(s)\, ds.\label{Numerics-eq-drJ}
\end{flalign}
\end{subequations}



