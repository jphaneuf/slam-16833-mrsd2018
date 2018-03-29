% CREATE_AB_NONLINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the A and b matrices for the 2D nonlinear SLAM problem
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l)
  %% Extract useful constants which you may wish to use
  n_poses =     size ( odom, 1 ) + 1; % +1 for prior on the first pose
  n_landmarks = max  ( obs( : , 2 ) );

  n_odom = size(odom, 1);
  n_obs  = size(obs, 1);

  % Dimensions of state variables and measurements (all 2 in this case)
  p_dim = 2;                                  % pose dimension
  l_dim = 2;                                  % landmark dimension
  o_dim = size(odom, 2);                      % odometry dimension
  m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

  % A matrix is MxN, b is Mx1
  N = p_dim *   n_poses + l_dim * n_landmarks;
  M = o_dim * ( n_odom  + 1 ) + m_dim * n_obs; % +1 for prior on the first pose

  %% Initialize matrices
  A = zeros(M, N);
  b = zeros(M, 1);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Ai definition : rx1 ry1 rx2 ry2 ... rxN ryN ... %%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%  l1x1 l1y1 l2x1 l2y1 ... lkxN lkxM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% Where rxi is robot x position at time i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%% ryi is robot y position at time i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%lkxi is x position of landmark k at time i %%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%lkyi is y position of landmark k at time i %%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Formulate odom section of A ( Same as linear example ) %%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  io  = 1: ( o_dim*n_odom         );
  iox = 1: ( o_dim*n_odom + o_dim );

  sigma_o =  1 / sqrt ( sigma_o ( 1 ) ); %% hack assuming symetric , diagonal matrix 
  sigma_l = 1  / sqrt ( sigma_l ( 1 ) ); %% 


  As = sparse (  io + o_dim , io  ,  -sigma_o * ones ( 1 , length ( io  ) ) , M , N ) + ... %rx/ry at t = -1
       sparse (  iox        , iox ,   sigma_o * ones ( 1 , length ( iox ) ) , M , N ); %     

  b ( o_dim + 1 : o_dim * n_odom + o_dim ) = sigma_o * odom ( 1 : o_dim*n_odom );

  for o = 0 : n_obs -1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Extract values from observation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i   = obs ( o + 1 , 1 ); % pose i at observation o
    l   = obs ( o + 1 , 2 ); % landmark at obseravation o
    lth = obs ( o + 1 , 3 ); % delta x to landmark
    ld  = obs ( o + 1 , 4 ); % delta y to landmark

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Define matrix offsets %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    obs_offset  = o_dim * ( n_odom + 1 ) + l_dim * o        ;
    lm_off      = o_dim * ( n_odom + 1 ) + l_dim * ( l - 1) ;
    pose_offset = p_dim * ( i      - 1 )                    ;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute jacobian subsection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rx = x ( pose_offset     + 1 );
    ry = x ( pose_offset     + 2 );
    lx = x ( lm_off + 1 );
    ly = x ( lm_off + 2 );
    H_sub = meas_landmark_jacobian( rx , ry , lx , ly );

    dtheta_dx =  H_sub ( 1 , 1 );
    dtheta_dy =  H_sub ( 1 , 2 );
    dd_dx     =  H_sub ( 2 , 1 );
    dd_dy     =  H_sub ( 2 , 2 );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Predict measurement based on x %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    h = meas_landmark( rx , ry , lx , ly );
    theta_p = h ( 1 );
    d_p     = h ( 2 );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Add landmark jacobian info to As %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    As = As + sparse (  obs_offset + 1 , lm_off + 1 , sigma_l * dtheta_dx , M,N );
    As = As + sparse (  obs_offset + 1 , lm_off + 2 , sigma_l * dtheta_dy , M,N );
    As = As + sparse (  obs_offset + 2 , lm_off + 1 , sigma_l * dd_dx     , M,N );
    As = As + sparse (  obs_offset + 2 , lm_off + 2 , sigma_l * dd_dy     , M,N );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Formulate b ( error vector ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    b ( obs_offset + 1 ) = sigma_l * ( wrapToPi ( lth - theta_p ) );
    b ( obs_offset + 2 ) = sigma_l * ( ld  - d_p     );
  end

end
