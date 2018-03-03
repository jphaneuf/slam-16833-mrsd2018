%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

%==== TEST: Setup uncertianty parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
%%This is wrong dudes:
%control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
control_cov = diag([ sig_r2, sig_beta2 ]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====
% Write your code here...
k = length ( measure ) /2 ;% number of locations we are tracking
landmark = get_landmark_locations ( pose , measure );
%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];
xcovs = ones ( 1 , k ) * sig_x2;
ycovs = ones ( 1 , k ) * sig_y2;
landmark_cov = [ xcovs ; ycovs ];
landmark_cov = diag ( landmark_cov ( : ) );

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    %Ok. We expect the robot's global position to change as a function of U
    % We do not expect the landmarks' positions to change.  
    % So we predict robot pose only at this stage
    % Same goes for robot pose covariance
    xr = x ( 1 : 3 );         % Extract robot pose estimate
    x_pre_robot = get_fxu ( xr , [ d , alpha ] );
    x_pre = x;
    x_pre ( 1 : 3 ) = x_pre_robot;

    % our prediction is not linear. So we want to get a state transition matrix Fx
    % We can get this by evaluating the jacobian d f (x , u ) / d x
    % Then we can project our covariance P forwards in time P = Fx P transpose ( Fx)
    % We also also want to add uncertainty in state space based on our control noise
    % We can define Fu as the jacobian d f( x , u ) / d u  , 
    %and add Fu measure_cov transpose (Fu) to our covariance prediction
    Fx = get_dfxu_dx ( x , [ d , alpha ] );
    Fu = get_dfxu_du ( x , [ d , alpha ] ); %lolz
    Pr = P ( 1 : 3 , 1 : 3 ); % Extract robot pose covariance
    Pr = Fx * Pr * Fx' + Fu * control_cov * Fu';
    P_pre = P;
    P_pre ( 1 : 3 , 1 : 3 )  = Pr ;
    % We now want to update the covariance matrix robot pose vs landmark position
    % Per SLAM EKF reference: http://ais.informatik.uni-freiburg.de/teaching/ss16/robotics/slides/13-slam.pdf slide 22
    % We should take the robot/landmark covariance and project by Fx
    for i = 0 : k -1
      % x ,  y rows. x , y columns for landmark i
      x_index = 4 + 2 * i;
      y_index = x_index + 1;
      sigLi = P_pre ( 1 : 3 , x_index : y_index );
      sigLi ( 3 , 3 ) = 0; % fake theta so we can use Fx
      sigLi = Fx * sigLi;
      sigLi = sigLi ( 1 : 3 , 1 : 2 );
      P_pre ( 1 : 3 , x_index : y_index ) = sigLi ;
      P_pre ( x_index : y_index , 1 : 3 ) = sigLi';
    end
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
%==== Close data file ====
fclose(fid);
