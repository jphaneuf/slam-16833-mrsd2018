% MEAS_LANDMARK_JACOBIAN
% 16-831 Fall 2016 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  x =  lx - rx;
  y =  ly - ry;
  dtheta_dx = - ( y ) / ( x^2 + y^2 );
  dtheta_dy =   ( x ) / ( x^2 + y^2 );
  dd_dx     = x / sqrt  ( x^2 + y^2 );
  dd_dy     = y / sqrt  ( x^2 + y^2 );
  H ( 1 , 1 ) = dtheta_dx;
  H ( 1 , 2 ) = dtheta_dy;
  H ( 2 , 1 ) = dd_dx;
  H ( 2 , 2 ) = dd_dy;
end
