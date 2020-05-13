# Optimal Actuator Location for Railway Tracks
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
<p align="center"><img src="/tex/1f340a6747f2de951da8c7c323dcb3c2.svg?invert_in_darkmode&sanitize=true" align=middle width=533.57751975pt height=59.178683850000006pt/></p>
The fitst order optimality conditions were derived in chapter 3. The optimality conditions use the derivative of the cost function with respect to the input and the actuator location. In that, the adjoint of the IVP needs to be calculated. The adjoint equation is the following final value problem (FVP):
<p align="center"><img src="/tex/a0474b8317a9933910d85b1ba431868b.svg?invert_in_darkmode&sanitize=true" align=middle width=484.43430584999993pt height=20.52955905pt/></p>
For every <img src="/tex/83788cfb666cb6880ad6a5126f543581.svg?invert_in_darkmode&sanitize=true" align=middle width=35.68482224999999pt height=22.831056599999986pt/>, the derivatives of the cost function with respect to <img src="/tex/322fd42358a5933530291efc5cb4cf6f.svg?invert_in_darkmode&sanitize=true" align=middle width=8.21920935pt height=14.15524440000002pt/> and <img src="/tex/ad00c578a101b09df3981ed7822da4a6.svg?invert_in_darkmode&sanitize=true" align=middle width=8.21920935pt height=14.15524440000002pt/> evaluated at <img src="/tex/5b3ed6474c6467af7484edb94101f082.svg?invert_in_darkmode&sanitize=true" align=middle width=73.96433549999999pt height=24.65753399999998pt/>, <img src="/tex/23ffebb98b4e00f49a2f1d59a683bc1f.svg?invert_in_darkmode&sanitize=true" align=middle width=76.70180924999998pt height=24.65753399999998pt/> are linear operators <img src="/tex/a7fe1ede9f3839b4db1952843489f9a2.svg?invert_in_darkmode&sanitize=true" align=middle width=57.561612899999986pt height=24.65753399999998pt/> and <img src="/tex/891b681e0180c0f90b985addd60cae17.svg?invert_in_darkmode&sanitize=true" align=middle width=59.89952429999999pt height=24.65753399999998pt/>, respectively. Identifying each operator with an element of its underlying space, they are derived as
<p align="center"><img src="/tex/d2a5a9e3f7a518a73f3257a27843ff2a.svg?invert_in_darkmode&sanitize=true" align=middle width=449.19430379999994pt height=65.9730753pt/></p>



