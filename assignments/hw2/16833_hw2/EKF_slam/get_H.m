%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluate H at( state ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ H ] = get_H ( state ) 
  %hardcoding some stuff, can't add landmarks 
  syms x y theta lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4 lx5 ly5 lx6 ly6;
  state_sym = [ x y theta lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4 lx5 ly5 lx6 ly6 ] ;
  hl = get_hl_symbolic();
  H_sym = jacobian ( hl , state_sym );
  H = H_sym;
  s = state; %shorthand
  H = subs ( H_sym , ...
     {  x    y theta  lx1  ly1  lx2  ly2  lx3  ly3  lx4   ly4   lx5   ly5   lx6   ly6 },...
     {s(1) s(2) s(3) s(4) s(5) s(6) s(7) s(8) s(9) s(10) s(11) s(12) s(13) s(14) s(15)} );
end
