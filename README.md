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
Observability Ranc Condition for Nonlinear systems can be calculated using following MATLAB script. More information can be found in paper:
<pre>
<code class="matlab">

   U= sym('U',[3,7]); %input-space
    x =sym('x',[7,1]); %state-space
dy = nonlin_eq_VSA(x,U(:,1), sys) ; %nonlinear dynamics of the system     
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

% find time derivatives of each sensors independently to fill the Y
for ind1=1:4
    for ind2=2:7
        Y(ind2,ind1) = lder(Y(ind2-1,ind1),dy,x);
        for ind3=1:ind2
            Y(ind2,ind1) =  Y(ind2,ind1) + lder(Y(ind2-1,ind1),U(:,ind2),U(:,ind2-1));
        end
    end
end
% 
K=sym(zeros(7,7,4)); 
for ind1=1:4
    K(:,:,ind1) = jacobian(Y(:,ind1),x);  % for each sensors create the observability matrix  independently
end

%%
%to find observability of combination of sensor 1 and 2:
rank([K(:,:,1); K(:,:,2)])
</code>
</pre>
