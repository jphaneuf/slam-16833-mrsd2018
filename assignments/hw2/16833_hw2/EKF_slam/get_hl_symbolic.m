% for some state p and control u at time t, predict state at next time step
%Note: we're using a time step of 1 , so disregarding timestep. Note generalizable.

function [ h ] = get_hl_symbolic ()
  % robot x y theta , landmark x y
  %hardcoding some stuff, can't add landmarks 
  syms x y theta lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4 lx5 ly5 lx6 ly6 ;
  n_landmarks = 6;
  landmarks   = [ lx1 ly1; lx2 ly2; lx3 ly3; lx4 ly4; lx5 ly5; lx6 ly6 ] ;
  h = []
  for i = 1 : n_landmarks
    lx     = landmarks ( i , 1 );
    ly     = landmarks ( i , 2 );
    hi     = get_hl_single_symbolic ( lx , ly );
    h = [ h ; hi ];
  end
end
