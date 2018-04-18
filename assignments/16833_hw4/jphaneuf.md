# 16-833 SLAM Homework 3
# Joe Phaneuf ( andrew id: jphaneuf)
# March 29 2018
## Collaborators : Adam Driscoll (jdriscol) , David Evans ( dje1 )  

#Introduction  
This homework demonstrates a linear and non-linear least squares approach to SLAM. Please see accompanying matlab code for implementation.  

# 1.  
## a.  
Point-to-point Iterative Closest Point defines an energy function to minimize in terms of Euclidean distance between a point, and a point at a subsequent timestep having undergone some transformation. This yields an estimate of the transformation. 

Point-to-plane Iterative Closest Point defines an energy function to minimize the dot product of a normal vector estimate and the difference between a vertex and a transformed vertex at a subsequent timestep. This also yields an estimate of the transformation, with an emphasis on maintaining surfaces.

##b.
derive equation 20: 
iterate
Want to estimate 
$$
\tilde{T}_{g,k}
$$
At each step we have an estimated pose.
Which gets used to estimate vertices and normals in the world frame.
At each subsequent step,
we use the new vertices, and solve for a transform that minimizes an error function ( point plane energy ).

This problem is nonlinear, so we linearize it by assuming small frame changes minimizing delta transformations.
$to frame movement, and try to find an incremental transform
$\

For landmark m, the measurement function at timestep i is:
$$
h_{mi} =
\begin{bmatrix}
\Delta x \\
\Delta y \\
\end{bmatrix}
=
\begin{bmatrix}
lx_{mi} - rx_{i} \\
ly_{mi} - ry_{i}
\end{bmatrix}
$$
Where  $lx_{m}$ , $ly_{m}$ , $rx$ , and $ry$ are landmark m x , y and robot pose x ,y values respectively.  

The jacobian of $h_{mi}$ with respect to state vector $x$ is:  
$$
H_{mi} =
\frac { \partial h_{mi} } { \partial x } =
\begin{bmatrix}
-1 &  0 & ... & 1 & 0 \\
 0 & -1 & ... & 0 & 1
\end{bmatrix}
\begin{bmatrix}
rx_{i} \\
ry_{i} \\
.\\.\\.\\
lx_{mi} \\
ly_{mi}
\end{bmatrix}
$$


The odometry measurement function at timestep i is:
$$
h_{i} =
\begin{bmatrix}
\Delta x \\
\Delta y \\
\end{bmatrix}
=
\begin{bmatrix}
rx_{i} - rx_{i-1} \\
ry_{i} - ry_{i-1}
\end{bmatrix}
$$

The jacobian of the odometry measurement function is:
$$
H_{i} =
\frac { \partial h_{i} } { \partial x } =
\begin{bmatrix}
-1 &  0 & 1 & 0 \\
 0 & -1 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
rx_{i-1} \\
ry_{i-1} \\
rx_{i}   \\
ry_{i}
\end{bmatrix}
$$

# 1.  
## c.  
### iii.  
I experimented with the economy parameter in the qr solver, and found that the results were not negatively affected when used. This is because the A matrix is very much over-determined, with many more rows than the columns. There is enough information to solve without using all observations.
 
## d.  
### i. 2D linear dataset  

Timing Results  
Chol2: 2.924090e-01 sec  
QR2:   1.204332e+00 sec  
Chol1: 1.910196e+00 sec  
QR1:   1.993215e+00 sec  
Pinv:  2.216453e+00 sec  
  
Pinv solution is correct!  
Chol1 solution is correct!  
Chol2 solution is correct!  
QR1 solution is correct!  
QR2 solution is correct!  

The Cholesky  with fill in performed the fastest. Cholesky will in general perform faster than QR decomposition, at the expense of numerical stability. That is seen here, with Chol1/Chol2 beating QR1/QR2. Chol2/QR2 perform faster than Chol1/QR1 as they adjust the fill in of the sparse matrices to speed up computation. The Pinv method does exploit the sparseness of these matrices, and consequently runs much slower. Figure 1 shows the sparsity patterns for each composition, and Figure 2 shows the final recovered trajectory.

\
![](./img/1di_spy.png)
Figure 1: 2D linear dataset matrix sparcity patterns
\

\
![](./img/1di_traj.png)
Figure 2: Trajectory and landmark positions 
\
  
  
### ii. 2D linear loop  

Timing Results
Chol2: 4.635700e-02 sec  
Pinv:  6.887600e-02 sec  
QR2:   2.130760e-01 sec  
QR1:   9.388670e-01 sec  
Chol1: 1.373674e+00 sec  
  
Pinv solution is too far off!  
Chol1 solution is too far off!  
Chol2 solution is too far off!  
QR1 solution is too far off!  
QR2 solution is too far off!  

Cholesky 2 again performs the best here. However pinv performs second best, unlike the last run, and the rest of the ordering is off. These solvers (aside from Pinv) rely on matrix and sparsity to speed up computation, and Figure 3 shows that the input matrices are much less sparse than in the previous data set, explaining the difference in computation time.  

Figure 4 shows the recovered trajectory. On this run, the landmarks and trajectory were not recovered with to the target accuracy ( 0.097 > 0.05 ), though everything is quite close.  

\
![](./img/1dii_spy.png)
Figure 3: 2d linear loop dataset 
\

\
![](./img/1dii_traj.png)
Figure 4: Trajectory and landmark positions loop
\  
    
# 2.  
## b.  
For convenience define:  
$$
\begin{matrix}
x = lx - rx
y = ly - ry
\end{matrix}
$$  

The jacobian of the non-linear landmark measurement function is:
$$
H_{i} =
\frac { \partial h_{mi} } { \partial x } =
\begin{bmatrix}
\frac { \partial \theta } { \partial rx} &
\frac { \partial \theta } { \partial ry} &
\frac { \partial \theta } { \partial lx} &
\frac { \partial \theta } { \partial ly} \\
\frac { \partial d } { \partial rx} &
\frac { \partial d } { \partial ry} &
\frac { \partial d } { \partial lx} &
\frac { \partial d } { \partial ly}
\end{bmatrix} =
$$

$$
\begin{bmatrix}
\frac {   y }  { x^{2} + y^{2} } &
\frac { - x }  { x^{2} + y^{2} } &
... &
\frac { - y }  { x^{2} + y^{2} } &
\frac {   x }  { x^{2} + y^{2} } \\
\frac { - x }  { \sqrt { x^{2} + y^{2} } } &
\frac { - y }  { \sqrt { x^{2} + y^{2} } } &
... &
\frac {   x }  { \sqrt { x^{2} + y^{2} } } &
\frac {   y }  { \sqrt { x^{2} + y^{2} } }
\end{bmatrix}
\begin{bmatrix}
rx_{i} \\
ry_{i} \\
.\\.\\.\\
lx_{mi} \\
ly_{mi}
\end{bmatrix}
$$  
   
## d.  
Figure 5 shows the trajectory against the non-linear dataset ( using the Chol2 for a linear solver ). The algorithm completed with an rms error of 0.0153 , performing much better than the raw odometry with rms error of 0.0579.
\
![](./img/2d_traj.png)
Figure 5: Trajectory and landmark positions non-linear
\  
  
  
## e.  
The non-linear SLAM algorithm had to be implemented incrementally, whereas the linear version dit not. This is because the change in error function as a function of state change cannot be encapsulated in linear form, so it must be approximated. The approximation is only valid close to the point of the linearization, so must be re-calculated when the state is changed.  
