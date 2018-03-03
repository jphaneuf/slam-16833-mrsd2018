%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ predicted_state ] = get_fxu ( p , u )
  syms x y theta vel drot;
  predicted_state_sym = get_fxu_symbolic();

  px     = p(1);
  py     = p(2);
  ptheta = p(3);
  uvel   = u(1);
  udrot  = u(2);

  predicted_state = subs ( predicted_state_sym , ...
                          {  x ,  y ,  theta ,  vel ,  drot } , ...
                          { px , py , ptheta , uvel , udrot } );
end
