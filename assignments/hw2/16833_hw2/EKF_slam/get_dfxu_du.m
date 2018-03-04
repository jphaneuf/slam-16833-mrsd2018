% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ V ] = get_dfxu_du ( p , u  )
  syms x y theta vel drot;
  Vsym = get_dfxu_du_symbolic ( );
  x     = p ( 1 );
  y     = p ( 2 );
  theta = p ( 3 );
  vel   = u ( 1 );
  drot  = u ( 2 );
  V = eval ( Vsym );
end
