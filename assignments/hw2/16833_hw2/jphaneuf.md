# 16-833 SLAM Homework 2
# Joe Phaneuf ( andrew id: jphaneuf)
# March 2 2018

# Introduction

# 1.
## a.
For a state P and control input u , our robot's positions at the next time step can be described like so :
$$
P = 
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
=
\begin{bmatrix}
p_{1} \\
p_{2} \\
p_{3}
\end{bmatrix}
$$

$$
u = 
\begin{bmatrix}
d_{t}\\
alpha_{t}
\end{bmatrix}
=
\begin{bmatrix}
    u_{1} \\
    u_{2}
\end{bmatrix}
$$

$$
\bar{P} = f ( P , u ) =
\begin{bmatrix}
p_{1} + u_{1} cos ( p_{3} ) \\
p_{2} + u_{1} sin ( p_{3} ) \\
p_{3} + u_{2}
\end{bmatrix}
$$


## b.
Say that our estimate of uncertainty at time t is 3x3 covariance matrix P. We define our our process noise covariance matrix Q as  
$$
Q=
\begin{bmatrix}
\sigma_{x}^{2} & 0 & 0 \\
0 & \sigma_{y}^{2} & 0 \\
0 & 0 & \sigma_{\alpha}^{2}
\end{bmatrix}
$$

To predict unceartainty at time t+1 , we project P forwards in time with state transition matrix F, then add process noise Q.
$$
P_{t+1} = F P_{t} F^{T} + Q
$$

Alternatively, we can ( and will ) create a 2x2 covariance matrix in control space.
$$
controlCov =
\begin{bmatrix}
\sigma_{d_{t}}^{2} & 0 \\
0 & \sigma_{\alpha_{t}}^{2} 
\end{bmatrix}
$$

controlCov can then be projected into state space using the Jacobian 


$$
F_{u} = \frac { \partial f ( \bar { P } , u ) } { \partial u }
$$

Resulting in a covariance prediction at time t+1:
$$
P_{t+1} = F P_{t} F^{T} + F_{u} controlCov F_{u}^{T}
$$


## c. 
Given a state estimate $p_{t}$ and a measurement $z$ , we find a landmark's global position $\bar{L}$ as follows.
$$
z = \begin{bmatrix}
    r     \\
    \beta 
    \end  {bmatrix}
=   \begin{bmatrix}
    z_{1} \\
    z_{2} 
    \end  {bmatrix}
$$
$$
\bar{L} = 
\begin{bmatrix}
l_{x}\\
l_{y}\\
\end{bmatrix}
= h ( p_{t} , z) =
\begin{bmatrix}
x + r cos ( \theta + \beta ) \\
y + r sin ( \theta + \beta )
\end{bmatrix} =
\begin{bmatrix}
p_{1} + z_{1} cos ( p_{3} + z_{2} ) \\
p_{2} + z_{1} sin ( p_{3} + z_{2} )
\end{bmatrix}
$$

The measurement uncertainty is characterized by covariance matrix R

$$
R =
diag (
\begin{matrix}
\sigma_{\beta 1}^{2} &
\sigma_{r 1}^{2} & 
\end{matrix}
)
$$

We can project the measurement covariance matrix into state space using the the landmark position function $\bar{L}$ defined in c and a measurement vector $z$.  
$$
V = \frac { \partial \bar { L } ( \bar { P } , z ) } { \partial z }
$$
$$
COV_{landmark} = V R V^{T}
$$

We then say we estimate the position of a landmark to be $\bar{L}$ with covariance $COV_{landmark}$

## d.
For some state $p$ we estimate the angle and range measurements 
$$
h ( p ) =
\begin{bmatrix}
\beta \\
r 
\end{bmatrix}
=
\begin{bmatrix}
wrapToPi( atan2 ( \frac { l_{y} - y } { l_{x} - x }  ) - \theta ) \\
\sqrt { ( l_{y} - y )^{2} + ( l_{x} - x)^{2} ) } 
\end{bmatrix}
$$

With covariance R ( defined in c. )

##e.

$$
H = \frac { \partial h } {\partial p}
= \begin{bmatrix}
\frac { \partial h_{1} } { \partial p_{1} } & \frac { \partial h_{1} } { \partial p_{2} } & \frac { \partial h_{1} } { \partial p_{3} } \\
 & & \\
\frac { \partial h_{2} } { \partial p_{1} } & \frac { \partial h_{2} } { \partial p_{2} } & \frac { \partial h_{2} } { \partial p_{3} }
  \end{bmatrix}
$$
$$
=
\begin{bmatrix}
\frac {1} { (l_{x} - x )^{2} + (l_{y} - y )^{2} ) } &
\frac {-1} { 1 + \frac {  (l_{y} - y )^{2} ) } { (l_{x} - x )^{2} } } &
-1 \\
\frac {-2 ( l_{x} - x ) } {\sqrt { (l_{x} - x )^{2} + (l_{y} - y )^{2} } } &
\frac {-2 ( l_{y} - y ) } {\sqrt { (l_{x} - x )^{2} + (l_{y} - y )^{2} } } &
0 
\end{bmatrix}
$$


##f.
$$
H = \frac { \partial h } {\partial l}
= \begin{bmatrix}
\frac { \partial h_{1} } { \partial l_{1} } & \frac { \partial h_{1} } { \partial l_{2} } \\
\frac { \partial h_{2} } { \partial l_{1} } & \frac { \partial h_{2} } { \partial l_{2} }
  \end{bmatrix}
=
\begin{bmatrix}
\frac {-1} { (l_{x} - x )^{2} + (l_{y} - y )^{2} ) } &
\frac {1} { 1 + \frac {  (l_{y} - y )^{2} ) } { 1 + (l_{x} - x )^{2} } } \\
\frac {2 ( l_{x} - x ) } {\sqrt { (l_{x} - x )^{2} + (l_{y} - y )^{2} } } &
\frac {2 ( l_{y} - y ) } {\sqrt { (l_{x} - x )^{2} + (l_{y} - y )^{2} } } 
\end{bmatrix}
$$

#2

## a.
There are 6 [ $\beta_{i} r_{i}$ ] pairs per line in the data file, each pair corresponding to a landmark.

## b.

