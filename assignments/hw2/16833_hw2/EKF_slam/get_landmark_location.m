function [ lx ly ] = get_landmark_location  ( p , z )
  %For some point p and single scan z ( with range r and angle Beta )
  % Return x and y coordinates of landmark
  x     = p(1);
  y     = p(2);
  theta = p(3);
  Beta  = z(1);
  r     = z(2);
  lx = x + r * cos ( theta + Beta );
  ly = y + r * sin ( theta + Beta );
end
