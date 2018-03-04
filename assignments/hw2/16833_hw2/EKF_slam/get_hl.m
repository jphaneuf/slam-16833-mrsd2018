%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Evaluate symbolic h ( state, position ) vector given entire state %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ h ] = get_hl (state) 
  % robot x y theta , landmark x y
  %hardcoding some stuff, can't add landmarks 
  syms x y theta lx1 ly1 lx2 ly2 lx3 ly3 lx4 ly4 lx5 ly5 lx6 ly6;
  s = state; %shorthand
  h_sym = get_hl_symbolic ();

  x     = s(1);
  y     = s(2);
  theta = s(3);
  lx1   = s(4);
  ly1   = s(5);
  lx2   = s(6) ;
  ly2   = s(7);
  lx3   = s(8);
  ly3   = s(9);
  lx4   = s(10);
  ly4   = s(11);
  lx5   = s(12);
  ly5   = s(13);
  lx6   = s(14);
  ly6   = s(15);
  h = eval ( h_sym );
  h ( 1 : 2 : end ) = wrapToPi ( h ( 1 : 2 : end ) )

end
