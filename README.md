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
