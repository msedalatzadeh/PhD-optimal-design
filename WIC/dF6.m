function dF = dF6(in1,in2)
%DF6
%    DF = DF6(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    17-Jun-2018 12:40:28

EI = in2(:,1);
alpha = in2(:,5);
k = in2(:,3);
l = in2(:,4);
rhoa = in2(:,2);
z1 = in1(1,:);
z2 = in1(2,:);
z3 = in1(3,:);
z4 = in1(4,:);
z5 = in1(5,:);
z6 = in1(6,:);
t2 = sparse(l.^2);
t3 = sparse(t2.^2);
t4 = sparse(k.*t3);
t5 = sparse(EI.*7.890136373754196e3);
t6 = sparse(rhoa.*t3.*7.890136373754196e3);
t7 = sparse(t4+t5+t6);
t8 = sparse(1.0./t7);
t9 = sparse(EI.*9.740909103400242e1);
t10 = sparse(rhoa.*t3.*9.740909103400242e1);
t11 = sparse(t4+t9+t10);
t12 = sparse(1.0./t11);
t13 = sparse(EI.*1.558545456544039e3);
t14 = sparse(rhoa.*t3.*1.558545456544039e3);
t15 = sparse(t4+t13+t14);
t16 = sparse(1.0./t15);
t17 = sparse(4.13961212179067e7);
t18 = sparse(3.725650909611603e8);
t19 = sparse(1.0./sqrt(t7));
t20 = sparse(1.0./sqrt(t11));
t21 = sparse(z5.^2);
t22 = sparse(l.*t2.*t8.*t21.*1.217849381327861e35);
t23 = sparse(z6.^2);
t24 = sparse(l.*t2.*t8.*t23.*1.217849381327861e35);
t25 = sparse(z1.^2);
t26 = sparse(l.*t2.*t12.*t25.*1.826774071991791e35);
t27 = sparse(z2.^2);
t28 = sparse(l.*t2.*t12.*t27.*1.826774071991791e35);
t29 = sparse(z3.^2);
t30 = sparse(l.*t2.*t16.*t29.*1.217849381327861e35);
t31 = sparse(z4.^2);
t32 = sparse(l.*t2.*t16.*t31.*1.217849381327861e35);
t33 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z1.*z6.*7.896447829498177e18);
t34 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z2.*z5.*7.896447829498177e18);
t50 = sparse(l.*t2.*t8.*z5.*z6.*2.435698762655721e35);
t51 = sparse(l.*t2.*t12.*z1.*z2.*3.653548143983582e35);
t52 = sparse(l.*t2.*t16.*z3.*z4.*2.435698762655721e35);
t53 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z1.*z5.*7.896447829498177e18);
t54 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z2.*z6.*7.896447829498177e18);
t35 = sparse(t22+t24+t26+t28+t30+t32+t33+t34-t50-t51-t52-t53-t54);
t36 = sparse(2.965820800757861e6);
t37 = sparse(l.^(3.0./2.0));
t38 = sparse(l.^(1.1e1./2.0));
t39 = sparse(z3-z4);
t40 = sparse(t18.*t19.*t36.*t37.*z5.*3.158273408348595e2);
t41 = sparse(t17.*t20.*t36.*t37.*z1.*5.68489213502747e3);
t55 = sparse(t18.*t19.*t36.*t37.*z6.*3.158273408348595e2);
t56 = sparse(t17.*t20.*t36.*t37.*z2.*5.68489213502747e3);
t42 = sparse(t40+t41-t55-t56);
t43 = sparse(l.*t2.*t12.*t25.*9.741852031373464e18);
t44 = sparse(l.*t2.*t12.*t27.*9.741852031373464e18);
t45 = sparse(l.*t2.*t16.*z3.*z4.*1.948370406274693e19);
t46 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z1.*z6.*2.526618726678876e3);
t47 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z2.*z5.*2.526618726678876e3);
t57 = sparse(l.*t2.*t16.*t29.*9.741852031373464e18);
t58 = sparse(l.*t2.*t16.*t31.*9.741852031373464e18);
t59 = sparse(l.*t2.*t12.*z1.*z2.*1.948370406274693e19);
t60 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z1.*z5.*2.526618726678876e3);
t61 = sparse(l.*t2.*t17.*t18.*t19.*t20.*z2.*z6.*2.526618726678876e3);
t48 = sparse(t43+t44+t45+t46+t47-t57-t58-t59-t60-t61);
t49 = sparse(alpha.*t3.*t17.*t18.*t19.*t20.*t48.*8.752455694935333e-33);
t62 = sparse(l.*t2.*t12.*t25.*1.217849381327861e35);
t63 = sparse(l.*t2.*t12.*t27.*1.217849381327861e35);
t64 = sparse(l.*t2.*t16.*t29.*1.826774071991791e35);
t65 = sparse(l.*t2.*t16.*t31.*1.826774071991791e35);
t69 = sparse(l.*t2.*t12.*z1.*z2.*2.435698762655721e35);
t70 = sparse(l.*t2.*t16.*z3.*z4.*3.653548143983582e35);
t66 = sparse(t22+t24-t33-t34-t50+t53+t54+t62+t63+t64+t65-t69-t70);
t67 = sparse(t17.*t20.*t36.*t37.*z1.*1.421223033756868e3);
t71 = sparse(t17.*t20.*t36.*t37.*z2.*1.421223033756868e3);
t68 = sparse(t40-t55+t67-t71);
t72 = sparse(l.*t2.*t8.*t21.*3.653548143983582e35);
t73 = sparse(l.*t2.*t8.*t23.*3.653548143983582e35);
t74 = sparse(l.*t2.*t12.*t25.*2.435698762655721e35);
t75 = sparse(l.*t2.*t12.*t27.*2.435698762655721e35);
t76 = sparse(l.*t2.*t16.*t29.*2.435698762655721e35);
t77 = sparse(l.*t2.*t16.*t31.*2.435698762655721e35);
t79 = sparse(l.*t2.*t8.*z5.*z6.*7.307096287967163e35);
t80 = sparse(l.*t2.*t12.*z1.*z2.*4.871397525311443e35);
t81 = sparse(l.*t2.*t16.*z3.*z4.*4.871397525311443e35);
t78 = sparse(t72+t73+t74+t75+t76+t77-t79-t80-t81);
dF = sparse([1,2,3,4,5,6,1,2,3,4,5,6,1,2,3,4,5,6,1,2,3,4,5,6,1,2,3,4,5,6,1,2,3,4,5,6],[1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,5,5,6,6,6,6,6,6],[alpha.*t3.*t12.*t35.*(-2.399535423529816e-33),alpha.*t3.*t12.*t35.*(-2.399535423529816e-33),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),t49,t49,alpha.*t3.*t12.*t35.*(-2.399535423529816e-33),alpha.*t3.*t12.*t35.*(-2.399535423529816e-33),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),t49,t49,alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),alpha.*t3.*t16.*t66.*(-3.839256677647705e-32),alpha.*t3.*t16.*t66.*(-3.839256677647705e-32),alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),alpha.*t16.*t17.*t20.*t36.*t38.*t39.*t42.*(-2.728220011388892e-29),alpha.*t3.*t16.*t66.*(-3.839256677647705e-32),alpha.*t3.*t16.*t66.*(-3.839256677647705e-32),alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),t49,t49,alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t3.*t8.*t78.*(-9.718118465295754e-32),alpha.*t3.*t8.*t78.*(-9.718118465295754e-32),t49,t49,alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t16.*t18.*t19.*t36.*t38.*t39.*t68.*(-5.456440022777784e-29),alpha.*t3.*t8.*t78.*(-9.718118465295754e-32),alpha.*t3.*t8.*t78.*(-9.718118465295754e-32)],6,6);
