function [fusion_map, next_ref_data] = pointBasedFusion(fusion_map, input_data, tform, cam_param, sigma, ds_ratio, t)
    
    %==== TEST: Set parameters (square of point distance threshold) ====
    dist_th = 0.05; %5cm
    dist2_th = dist_th^2;
    dot_th = cos(20*pi/180); %20degrees

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    h = size(input_points, 1);
    w = size(input_points, 2);
    
    fprintf (" //////// \n fusion_map size : %d \n " , length ( fusion_map.pointcloud.Location ) );
    %==== Transform the input data for fusion ====
    %trans_pointcloud = pctransform(input_data.pointcloud, tform); 
    %trans_points     = trans_pointcloud.Location;
    %trans_normals    = nvRotate(input_data.normals, tform);
    %trans_data       = struct('pointcloud', trans_pointcloud, 'normals', trans_normals);

  
    %==== Project the global fusion map onto the input image frame ==== 
    [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param); 
    proj_points = proj_map.points;
    proj_normals = proj_map.normals;

    
    %==== Find if each projected map point and its corresponding input point are close in distance ====
    %is_close = isInputCloseToProjPoints(trans_points, proj_points, dist2_th);
    is_close = isInputCloseToProjPoints(input_data.pointcloud.Location, proj_points, dist2_th);
    
    %==== Find if each projected map point and its corresponding input point are similar in normal direction ====
    %is_similar = isInputSimilarToProjNormals(trans_normals, proj_normals, dot_th);
    is_similar = isInputSimilarToProjNormals(input_data.normals, proj_normals, dot_th);
    
    %==== Find if each input point is a newly observed point ====
    is_first = sum(proj_points.^2, 3) == zeros(h, w);
    
    %==== Determine if each input point should be added into the fusion map ====
    is_use = isUsableInputPoints(is_close, is_similar, is_first);
    
    fprintf (" %d useable points\n", sum ( sum ( is_use  ) ) );
    fprintf (" %d first time buyers \n" , sum ( sum ( is_first ) ) );
    fprintf (" %d close points  \n" , sum ( sum ( is_close ) ) );
    fprintf (" %d similar points (normal ) \n" , sum ( sum ( is_similar ) ) );
    %==== Calculate alpha[] based on the radial distance of each point to the camera center ==== 
    alpha = getAlpha(input_points, sigma);
    
    %==== Average the projected map with the input data ====
    %updated_map = avgProjMapWithInputData(proj_map, trans_data, alpha, h, w, is_use, t);
    updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t);

    %updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
    ptstmp = updated_map.points;
    ptstmp = reshape ( ptstmp , [ ] , 3 );
    ptstmp = ptstmp';
    ptstmp(4,:) = 1;
    updated_map.points = ( (tform.T') * ptstmp )';
    updated_map.points = updated_map.points ( : , 1:3);
    updated_map.points = reshape ( updated_map.points , h , w , 3 );
    updated_map.normals    = nvRotate(updated_map.normals, tform);



%{
    subplot ( 2 , 2 , 1 );
    showPointCloud ( input_points );
    view ( [ 0 1 0 ]  )
    title('input');
    subplot ( 2 , 2 , 2 );
    showPointCloud ( updated_map.points );
    view ( [ 0 1 0 ]  )
    title('updated points');
    subplot ( 2 , 2 , 3 );
    showPointCloud ( fusion_map.pointcloud.Location );
    view ( [ 0 1 0 ]  )
    title('fusion');
    subplot ( 2 , 2 , 4 );
    showPointCloud ( proj_points );
    view ( [ 0 1 0 ]  )
    title('proj');

    pause ( 1 );
  %}
 


    
    %==== Downsample the updated map to get the reference data for next ICP registraion ===
    next_ref_data = getNextRefData(updated_map, ds_ratio);
    
    %==== Merge the averaged projected map with the unchanged part of the original fusion map to get the final fusion map ====
    fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag);
      
end
