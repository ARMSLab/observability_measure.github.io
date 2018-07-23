<link rel="stylesheet" href="/observability_measure.github.io/default.css">
<script src="/observability_measure.github.io/highlight.pack.js"></script>
<script>hljs.initHighlightingOnLoad();</script>

<script type="text/x-mathjax-config">
MathJax.Hub.Config({
  tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}
});
</script>
<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-MML-AM_CHTML">
</script>



# Tutorial on Observability Measure
In this tutorial the observability measure based optimal sensor placement is considered. This tutorial shows the following steps for sensor placement problem:
1. What is sensor placement problem and observability measure. Key concepts.
2. How Observability is measured
3. How the sensor placement problem is solved

In this tutorial the observability is measured for Variable Stiffness Actuator System with following equation of dynamics
\begin{align}
\dot{x} &= f(x,u) \newline
y&=Cx+Du
\end{align}
`x` - states of the system
`u` - input
`y` - output

##Key concepts
Optimal Sensor placement problem is type of problem where the sensory devices are integrated to the system to satisfy some specified condition. In conventional robotic systems, sensors are placed to guarantee the full observability of the system. Observability of Linear System could be found [here](https://en.wikipedia.org/wiki/Observability). For nonliner systems, system could be unobservable for some specific set of configurations in the state-space. However, for some specific cases the system could have more than one fully observable sensor configurations. For such cases it is hard to gain intuition on which of observable states are "better". Therefore observability measure is introduced.

In this tutorial two different systems are presented
  1. VSA robot with reaction wheel
  2. Double linked VSA robot
Here, only VSA robot with reaction wheel is presented. However, you can find second one in downloads flies.

##Observability Measure
1) Observability Ranc Condition for Nonlinear systems can be calculated using following MATLAB script. More information can be found in paper:
<pre>
<code class="matlab">
U= sym('U',[3,7]); %input-space
x =sym('x',[7,1]); %state-space

dy = <a href="https://github.com/ARMSLab/observability_measure.github.io/blob/master/Observability/VSAwrw/nonlin_eq_VSA.m">nonlin_eq_VSA</a>(x,U(:,1), sys) ; %nonlinear dynamics of the system     
C=zeros(4,7);                       % create matrix of ouputs so that y=C*x
C(1,1)=1;
C(2,3)=1;
C(3,4)=1;
C(4,5)=1;
C=sym(C);

Y=sym(zeros(7,4));            %sensor-space y=[s1,...,s4; dot_s1,...,s4;...]
for ind=1:4
    Y(:,ind) = C(ind,:)*x;
end

% find time derivatives of each sensors independently to fill the Y.
%Seven's order derivative is sufficient.
for ind1=1:4
    for ind2=2:7
        Y(ind2,ind1) = <a href="https://github.com/ARMSLab/observability_measure.github.io/blob/master/Observability/VSAwrw/lder.m">lder</a>(Y(ind2-1,ind1),dy,x);
        for ind3=1:ind2
            Y(ind2,ind1) =  Y(ind2,ind1) + <a href="https://github.com/ARMSLab/observability_measure.github.io/blob/master/Observability/VSAwrw/lder.m">lder</a>(Y(ind2-1,ind1),U(:,ind2),U(:,ind2-1));
        end
    end
end
% 
K=sym(zeros(7,7,4)); 
for ind1=1:4
    K(:,:,ind1) = jacobian(Y(:,ind1),x);  %for each sensors create the observability matrix
end

%%
%to find observability of combination of sensor 1 and 2:
rank([K(:,:,1); K(:,:,2)])
</code>
</pre>

2) Observability Gramian can be calculated using two different ways: Empirical and Linearized. 
In linearized case, Gramian for nonlinear system calculated as linear one by linearization of the system and feeding the linearized matrix as following:

\begin{align}
        W(t) = \int^{t}_{0}\Phi^T_dC^TC\Phi(t) dt \newline
        \dot{\Phi(t)} = A(x(t),u(t))\Phi(t) \newline
        \Phi(0) = I
\end{align}

Implementation of this method can be found in tutorial. 
In empirical Gramian method, the Gramian is calculated by perturbing the states and calculation of the outputs:
\begin{align}
        \bar{x}^{\pm k}_0 = x_0 \pm \epsilon e_k  \newline
        y^{\pm i} & = h(\bar{x}^{\pm i}(t)) \newline
        W(i,j) &= \frac{1}{4\epsilon^2}\int^{T}_0 (y^{+i} - y^{-i})^T(y^{+j} - y^{-j})dt,
\end{align}
