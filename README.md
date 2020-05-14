# Optimal Actuator Location for Railway Track PDE
Actuator location and design are important design variables  in controller synthesis for distributed parameter systems. Finding the best actuator location to control a distributed parameter system can significantly reduce the cost of the control and improve  its effectiveness. From a theoretical point of view, the existence of an optimal actuator location has been discussed in the literature for linear partial differential equations (PDE's). It was proven that an optimal actuator location exists for linear-quadratic control.  Conditions under which using approximations in optimization yield the optimal location are also established.
 Similar results have been obtained for <img src="/tex/912631c954499428b64ab8d828ac8cb6.svg?invert_in_darkmode&sanitize=true" align=middle width=20.21695004999999pt height=22.465723500000017pt/> and <img src="/tex/d73a36c9f434bb145402252895d2ebb6.svg?invert_in_darkmode&sanitize=true" align=middle width=26.76948449999999pt height=22.465723500000017pt/> controller design objectives with linear models. There are also results on the related problem of optimal sensor location for linear PDE's.



## Raiway Track Model
Railway tracks are rested on ballast which are known for exhibiting nonlinear viscoelastic behavior. If a track beam is made of a Kelvin-Voigt material, then the railway track model will be a semi-linear partial differential equation on <img src="/tex/2a1afb1e2126e7a44c815592bdd86dd1.svg?invert_in_darkmode&sanitize=true" align=middle width=59.54613389999999pt height=24.65753399999998pt/> as follows:

<p align="center"><img src="/tex/1987f82bdf2498f1a755ed9f10f25010.svg?invert_in_darkmode&sanitize=true" align=middle width=594.7234622999999pt height=159.54863429999997pt/></p>

where the positive constants <img src="/tex/84df98c65d88c6adf15d4645ffa25e47.svg?invert_in_darkmode&sanitize=true" align=middle width=13.08219659999999pt height=22.465723500000017pt/>, <img src="/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/>, <img src="/tex/6dec54c48a0438a5fcde6053bdb9d712.svg?invert_in_darkmode&sanitize=true" align=middle width=8.49888434999999pt height=14.15524440000002pt/>, <img src="/tex/44bc9d542a92714cac84e01cbbb7fd61.svg?invert_in_darkmode&sanitize=true" align=middle width=8.68915409999999pt height=14.15524440000002pt/>, and <img src="/tex/d30a65b936d8007addc9c789d5a7ae49.svg?invert_in_darkmode&sanitize=true" align=middle width=6.849367799999992pt height=22.831056599999986pt/> are the modulus of elasticity, second moment of inertia, density of the beam, cross-sectional area, and length of the beam, respectively. The linear and nonlinear parts of the foundation elasticity correspond to the coefficients <img src="/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/> and <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, respectively. The constant <img src="/tex/b5efe2097e788f0aad8b0da8567e42cc.svg?invert_in_darkmode&sanitize=true" align=middle width=40.04176439999999pt height=21.18721440000001pt/> is the viscous damping coefficient of the foundation, and <img src="/tex/47a026a1db28d941893a8537e7e23dda.svg?invert_in_darkmode&sanitize=true" align=middle width=49.550684699999984pt height=22.465723500000017pt/> is the coefficient of Kelvin-Voigt damping in the beam.
The track deflection is controlled by an external force  <img src="/tex/633aafb63e9ef3f6da5d763d63ed3a95.svg?invert_in_darkmode&sanitize=true" align=middle width=28.13180369999999pt height=24.65753399999998pt/>;  <img src="/tex/633aafb63e9ef3f6da5d763d63ed3a95.svg?invert_in_darkmode&sanitize=true" align=middle width=28.13180369999999pt height=24.65753399999998pt/>  will  be
assumed to be a scalar input in order to simplify the exposition. The shape influence function <img src="/tex/89af0d915a632e01f8da1b3ef42527c8.svg?invert_in_darkmode&sanitize=true" align=middle width=42.96715664999999pt height=24.65753399999998pt/> is a continuous function over <img src="/tex/8250e61c2154c3ca2d3f307958bfd9dd.svg?invert_in_darkmode&sanitize=true" align=middle width=31.50690839999999pt height=24.65753399999998pt/> parametrized by the parameter <img src="/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/> that describes its dependence on actuator location. For example, as shown in the next figure, the control force is typically localized at some point <img src="/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/> and <img src="/tex/89af0d915a632e01f8da1b3ef42527c8.svg?invert_in_darkmode&sanitize=true" align=middle width=42.96715664999999pt height=24.65753399999998pt/> models the distribution of the force <img src="/tex/633aafb63e9ef3f6da5d763d63ed3a95.svg?invert_in_darkmode&sanitize=true" align=middle width=28.13180369999999pt height=24.65753399999998pt/> along the beam. The
function <img src="/tex/2b4416ce2e321aa9278e0f6e393fbfed.svg?invert_in_darkmode&sanitize=true" align=middle width=42.96715664999999pt height=24.65753399999998pt/>  is assumed continuously differentiable with respect to <img src="/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/> over <img src="/tex/f3e711926cecfed3003f9ae341f3d92b.svg?invert_in_darkmode&sanitize=true" align=middle width=11.87217899999999pt height=22.648391699999998pt/>; a suitable function for the case of actuator location is illustrated in the figure.

<p align="center">
<img src="figs/Beam-schematic-2.JPG" width="400" />
</p>













## Optimal Controller and Actuator Location
In this section, we apply the results of previous sections to compute an optimal control and actuator location for the vibration suppression of the track. As discussed in chapter 3, the problem of finding the best control and actuator location is the optimization problem
<p align="center"><img src="/tex/ed4bc13d16761e8e0785a107fd2ac38f.svg?invert_in_darkmode&sanitize=true" align=middle width=406.79243054999995pt height=69.0417981pt/></p>

The optimality conditions use the derivative of the cost function with respect to the input and the actuator location. In that, the adjoint of the IVP needs to be calculated. The adjoint equation is the following final value problem (FVP):
<p align="center"><img src="/tex/0b85743610579cf48baf2b3d0ddbc833.svg?invert_in_darkmode&sanitize=true" align=middle width=332.40180104999996pt height=17.2895712pt/></p>

For every <img src="/tex/794c6f1b3b189816a9973dda0d498a24.svg?invert_in_darkmode&sanitize=true" align=middle width=51.76927469999998pt height=22.465723500000017pt/>, the derivatives of the cost function with respect to <img src="/tex/6dbb78540bd76da3f1625782d42d6d16.svg?invert_in_darkmode&sanitize=true" align=middle width=9.41027339999999pt height=14.15524440000002pt/> and <img src="/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/> evaluated at <img src="/tex/b6fc862fdaf2fdac1381660401858678.svg?invert_in_darkmode&sanitize=true" align=middle width=87.94070669999998pt height=24.65753399999998pt/>, <img src="/tex/26f622d7f5bcda8339197d99bca8e680.svg?invert_in_darkmode&sanitize=true" align=middle width=89.14086224999998pt height=24.65753399999998pt/> are linear operators <img src="/tex/df15a679eaf6391c5cb3e0d7a268c5a1.svg?invert_in_darkmode&sanitize=true" align=middle width=94.34988914999998pt height=24.65753399999998pt/> and <img src="/tex/6e396c94cd46268fa84d584703049006.svg?invert_in_darkmode&sanitize=true" align=middle width=93.03514274999998pt height=24.65753399999998pt/>, respectively. Identifying each operator with an element of its underlying space, they are derived as

<p align="center"><img src="/tex/6a9c8732ef7332aa1d0dc82bda735756.svg?invert_in_darkmode&sanitize=true" align=middle width=285.01677435pt height=49.315569599999996pt/></p>





















## Optimization Algorithms
Several optimization algorithms were suggested in the literature for solving PDE-constrained optimization problems. In this section, two common optimization algorithms for solving the optimization problem will be discussed. These are projected gradient method and nonlinear conjugate gradient method. In projected gradient (or steepest descent) method, the negative of the gradient is considered as the search direction. 

#### Projected Gradient Algorithm
The projected gradient method reads as follows:

<p align="center"><img src="/tex/eb3978a97659368637046fd6e8d0ebc3.svg?invert_in_darkmode&sanitize=true" align=middle width=653.30609025pt height=307.3059396pt/></p>

Projected gradient method is typically converging to an optimizer slowly, whereas the nonlinear conjugate gradient method promises faster convergence \cite{nocedal1999}. 

#### Conjugate Gradient Algorithm
The nonlinear conjugate gradient method reads as follows:

<p align="center"><img src="/tex/446c9eef00c5c420ac599b79cbb821f7.svg?invert_in_darkmode&sanitize=true" align=middle width=688.4025555pt height=710.8838121pt/></p>



Several choices exist for selecting the step length <img src="/tex/6048c4fc08d21c02067a5da6a58c128b.svg?invert_in_darkmode&sanitize=true" align=middle width=34.067908049999986pt height=22.831056599999986pt/> (similarly <img src="/tex/2821c0570e6ce1f89053bac27705416b.svg?invert_in_darkmode&sanitize=true" align=middle width=34.067908049999986pt height=22.831056599999986pt/>) of the previous algorithm. Letting <img src="/tex/7d853684f81df818b709dc7fd758f3b1.svg?invert_in_darkmode&sanitize=true" align=middle width=128.77141364999997pt height=22.831056599999986pt/>, the following are considered for selecting the step length <img src="/tex/6048c4fc08d21c02067a5da6a58c128b.svg?invert_in_darkmode&sanitize=true" align=middle width=34.067908049999986pt height=22.831056599999986pt/> (similarly <img src="/tex/2821c0570e6ce1f89053bac27705416b.svg?invert_in_darkmode&sanitize=true" align=middle width=34.067908049999986pt height=22.831056599999986pt/>)

<p align="center"><img src="/tex/d13ff68f2e1e6d904e104920b341a293.svg?invert_in_darkmode&sanitize=true" align=middle width=299.10250755pt height=133.51988099999997pt/></p>



A new formula was also proposed by Hager and Zhang; define <img src="/tex/1c9d379fca0ba4abc98eeec665ac2dc7.svg?invert_in_darkmode&sanitize=true" align=middle width=34.067908049999986pt height=27.342470100000007pt/> and <img src="/tex/a8ccffe21d32d0177c73290cf2feface.svg?invert_in_darkmode&sanitize=true" align=middle width=32.932104149999994pt height=21.839370299999988pt/> as

<p align="center"><img src="/tex/90aff62e5f8cdc03683ac0931d2da560.svg?invert_in_darkmode&sanitize=true" align=middle width=283.2773097pt height=97.9161975pt/></p>



Then, the formula is
<p align="center"><img src="/tex/28cf6cb6d69167eba74e42db4c063dcc.svg?invert_in_darkmode&sanitize=true" align=middle width=299.01318315pt height=19.726228499999998pt/></p>




Furthermore, several schemes have been proposed to choose the step length <img src="/tex/a68ff5515c27e035477f850a94c55cee.svg?invert_in_darkmode&sanitize=true" align=middle width=15.831502499999988pt height=21.839370299999988pt/> (similarly <img src="/tex/764105dc279f1c31f3ecbf550438853a.svg?invert_in_darkmode&sanitize=true" align=middle width=15.831502499999988pt height=21.839370299999988pt/>) in each iteration of previous algorithms including bisection, (strong) Wolfe conditions, Secant method.

<p align="center"><img src="/tex/0ad484abc2c0010b52b4a1dfbbd41152.svg?invert_in_darkmode&sanitize=true" align=middle width=680.4113811pt height=878.9514062999999pt/></p>















## Simulation Results

In the railway track model, if the variables <img src="/tex/31fae8b8b78ebe01cbfbe2fe53832624.svg?invert_in_darkmode&sanitize=true" align=middle width=12.210846449999991pt height=14.15524440000002pt/>, <img src="/tex/4f4f4e395762a3af4575de74c019ebb5.svg?invert_in_darkmode&sanitize=true" align=middle width=5.936097749999991pt height=20.221802699999984pt/>, and <img src="/tex/85e60dfc14844168fd12baa5bfd2517d.svg?invert_in_darkmode&sanitize=true" align=middle width=7.94809454999999pt height=22.831056599999986pt/> are appropriately substituted with dimensionless variables, a dimensionless PDE model for the railway track can be derived. Moreover, the final interval time is set to <img src="/tex/533a256e00d4052bda65422cf4af0d07.svg?invert_in_darkmode&sanitize=true" align=middle width=47.40288299999998pt height=21.18721440000001pt/>. This gives the state of the controlled system enough time to settle. In addition, we choose <img src="/tex/b9221dc7c33afb6bfc7d9210029e3942.svg?invert_in_darkmode&sanitize=true" align=middle width=91.42108964999998pt height=26.76175259999998pt/> so that the input force is concentrated on a relatively small region on the track. In the cost function, we choose the same weights for the deflection, rate of deflection, and input; so <img src="/tex/6d09275a247d896ab33cdb5a7ae65b08.svg?invert_in_darkmode&sanitize=true" align=middle width=122.35896914999998pt height=21.18721440000001pt/> is selected for the simulations. 

Given an order of approximation, the initial conditions are chosen such that all modes are excited. The initial conditions are chosen from
<p align="center"><img src="/tex/00fad6a42a2b2bfe0c079a181e28b34b.svg?invert_in_darkmode&sanitize=true" align=middle width=290.795868pt height=39.452455349999994pt/></p>

The order of approximation is equal to the dimension of an initial condition. For example, if the order of approximation is 4, the initial condition is

<p align="center"><img src="/tex/5ddf5c4a25dde4850c273e14867b4bc1.svg?invert_in_darkmode&sanitize=true" align=middle width=141.0978162pt height=39.452455349999994pt/></p>

The initial condition is illustrated in for the 10th order approximation. 

<p align="center">
<img src="results/initial condition.JPG" width="400" />
</p>

Simulations were conducted using the software MATLAB. The ODE solver `ode15s` was used to solve the finite-dimensional approximation of the system. MATLAB optimization routine `fmincon` was also used as the optimization algorithm. The convergence of the approximation method is illustrated. It is observed that beyond 6th order approximation, increasing the approximation order will not make a noticeable difference. The following figure compare the cost and optimal input for the linear and nonlinear model in the presence and absence of damping. These figures indicate a significant change in the cost of control and in the optimal input. It also shows how the cost and optimal location of actuators change when the coefficient of nonlinearity, <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/>, is increased. As a general rule of thumb, increasing <img src="/tex/c745b9b57c145ec5577b82542b2df546.svg?invert_in_darkmode&sanitize=true" align=middle width=10.57650494999999pt height=14.15524440000002pt/> increases the cost of control. Moreover, it shows how the cost and location of actuators change when the coefficient of viscous and Kelvin-Voigt damping are decreased. Simulations show that the optimal location of actuators moves away from the center as the damping is decreased. Also, an interesting observation is made where local optimizers appear by decreasing the coefficient of Kelvin-Voigt damping. Lastly, it shows the improvement in the performance of the control system when the optimal location is chosen for the actuator over the center location.  


|Convergence of the numerical scheme for different orders of approximation in undamped beam. No significant improvement is observed for 4th order approximation or higher.|
|:----:|
|<p align="center">
<img src="results/increasing order.eps" width="400" />
</p>|



## References