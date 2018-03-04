% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ V ] = get_dfxu_dx ( p , u  )
  syms x y theta vel drot;

  x     = p ( 1 );
  y     = p ( 2 );
  theta = p ( 3 );
  vel   = u ( 1 );
  drot  = u ( 2 );

  Vsym = get_dfxu_dx_symbolic ( );
  V = eval ( Vsym );
  %V = subs ( Vsym , {  x ,  y ,  theta ,  vel ,  drot } , ...
  %                  { px , py , ptheta , uvel , udrot } );
end
% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.
