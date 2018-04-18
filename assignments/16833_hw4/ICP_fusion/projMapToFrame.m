function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Prepare transforms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tform = inv ( tform.T' ); %Heathens
    R =     inv ( tform ( 1:3 , 1:3 ) );
    R =     tform ( 1:3 , 1:3 );
    %
    tform = tform ( 1:3 , : ); %4x4 => 3x4 extrinsic camera parameter
    K = [ fx 0 cx ; 0 fy cy ; 0 0 1 ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% locations => homogeneous coordinates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    locs   = fusion_map.pointcloud.Location;
    locs_t = vertcat ( locs' , ones ( 1 , size ( locs , 1 ) ) );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Apply transform %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Use proj_points_full to get image coordinates%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Use tform_points to track point cloud%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cam_points_full  = tform * locs_t;
    proj_points_full = K * tform * locs_t;
    proj_points_full = proj_points_full';
    cam_points_full  = cam_points_full';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get indices of points in front of camera %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i_zpositive = find ( proj_points_full(:,3) > 0 );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Normalize and rount points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    proj_points_full = proj_points_full ./ proj_points_full ( : , 3 ); % normalize to z =  1
    proj_points_full = round ( proj_points_full );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get indices of unique points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [ unique_proj_points_full , i_unique , i_meh ] = unique( proj_points_full , 'rows');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get indices of inbounds points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_inbounds  = and ( proj_points_full ( : , 1) > 0  , proj_points_full ( : , 1 ) <= w );
    y_inbounds  = and ( proj_points_full ( : , 2) > 0  , proj_points_full ( : , 2 ) <= h );
    i_inbounds  = find ( and ( x_inbounds , y_inbounds ) );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Get total valid indices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    valid_indices = intersect ( i_inbounds ,  ...
                    intersect ( i_unique , i_zpositive ) );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Make ccounts, times, normals %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %sparse inputs with duplicate indices will increment...
    %% i.e. x = [ 1 2 3 1 ] , y = [ 4 5 6 4 ] , v = ones () , will produce 2,5 = 1  , 3,6 = 1 , 1,4 = 2
    %%%Points are stored x/y , matrices row column
    %proj_ccounts  = full ( sparse ( proj_points_full ( valid_indices , 2 ) , proj_points_full ( valid_indices , 1 ) , ones ( 1 , length ( valid_indices ) ) , h , w ) );
    proj_ccounts  = full ( sparse ( proj_points_full ( valid_indices , 2 ) , proj_points_full ( valid_indices , 1 ) , fusion_map.ccounts ( valid_indices ) , h , w ) );
    proj_times    = full ( sparse ( proj_points_full ( valid_indices , 2 ) , proj_points_full ( valid_indices , 1 ) , fusion_map.times   ( valid_indices ) , h , w ) );

    proj_normals_full = ( R * fusion_map.normals')';
    proj_points  = zeros ( h , w , 3 );
    proj_normals = zeros ( h , w , 3 );
    proj_colors  = zeros ( h , w , 3 );
    for i = valid_indices'
      x = proj_points_full ( i , 1 );
      y = proj_points_full ( i , 2 );
      proj_points  ( y , x , : ) = cam_points_full             ( i , 1:3 );
      proj_normals ( y , x , : ) = proj_normals_full           ( i ,  :  ) ;
      proj_colors  ( y , x , : ) = fusion_map.pointcloud.Color ( i ,  :  ) ;
    end
    fprintf ( 'valid projection indices: %d\n' , length ( valid_indices ) );
    proj_flag = valid_indices;
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
