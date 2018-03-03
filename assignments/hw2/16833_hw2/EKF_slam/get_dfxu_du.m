% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ V ] = get_dfxu_du ( p , u  )
  syms x y theta vel drot;
  px     = p(1);
  py     = p(2);
  ptheta = p(3);
  uvel   = u(1);
  udrot  = u(2);
  Vsym = get_dfxu_du_symbolic ( );
  V = subs ( Vsym , {  x ,  y ,  theta ,  vel ,  drot } , ...
                    { px , py , ptheta , uvel , udrot } );
end
