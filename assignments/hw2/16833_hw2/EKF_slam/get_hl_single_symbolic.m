% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ zi ] = get_hl_single_symbolic (lx,ly)
  % robot x y theta , landmark x y
  syms x y theta ; %lx ly ;
  %Beta = wrapToPi ( atan2 ( ( ly - y ) , ( lx - x ) )  - theta )
  Beta = atan2 ( ( ly - y ) , ( lx - x ) )  - theta;
  r = sqrt ( ( ly - y )^2 + ( lx - x )^2 ) ;
  zi = [ Beta ; r ];
end
