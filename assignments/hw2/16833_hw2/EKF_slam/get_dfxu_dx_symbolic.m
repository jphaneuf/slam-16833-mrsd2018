% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ F ] = get_dfxu_dx_symbolic (  )
  predicted_state = get_fxu_symbolic ( )
  syms x y theta 
  p = [ x y theta ]
  F = jacobian ( predicted_state , p ) 
end



%{
function [ state ] = state_transition_function ( p , u )
  state = zeros ( 3, 1 )
  x     = p(1);
  y     = p(2);
  theta = p(3);
  vel   = u(1);
  alpha = u(2);
  state ( 1 ) = x + vel * cos ( theta ) 
  state ( 2 ) = y + vel * sin ( theta ) 
  state ( 3 ) = theta + alpha
end
%}
