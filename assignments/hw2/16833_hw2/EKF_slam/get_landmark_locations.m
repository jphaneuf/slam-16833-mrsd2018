%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ landmarks ] = get_landmark_locations ( p , zs )
  %for some point p and set of landmark 
  % scans zs ( beta1 , r1, .... betan , rn )
  % return vector ( lx1 , ly1 , ... lxn , lyn)
  nmt = 2; % n measurement types , beta and r
  nm  = length ( zs ) / nmt;
  zs = reshape ( zs , [ nmt , nm ] )';
  landmarks = [];
  for i = 1 : nm  
    [ lx , ly ] = get_landmark_location ( p , zs ( i , :) );
    landmarks = [ landmarks ; lx ; ly  ];
  end


  %Can try to vectorize. This is what should work but doesn't:
  %zs = mat2cell ( zs , nm , nmt )
  %landmarks = cellfun ( ...
  %  @( x ) get_landmark_location( p , x )  , ...
  %     zs)
  %Not worth messing with garbage matlab
end
