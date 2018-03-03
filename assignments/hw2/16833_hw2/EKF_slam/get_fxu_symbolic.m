% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ predicted_state ] = get_fxu_symbolic ()
  syms x y theta vel drot;
  predicted_state ( 1 ) = x + vel * cos ( theta ) ;
  predicted_state ( 2 ) = y + vel * sin ( theta ) ;
  predicted_state ( 3 ) = theta + drot;
end
