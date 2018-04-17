function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...

    map_points  = fusion_map.pointcloud.Location;
    map_colors  = fusion_map.pointcloud.Color;
    map_normals = fusion_map.normals;
    map_ccounts = fusion_map.ccounts;
    map_times   = fusion_map.times;
    for i = proj_flag'
      [ r  c  ] = ind2sub ( [ h , w ] , i );
      map_points  ( i , : ) = updated_map.points  ( r , c , : );
      map_colors  ( i , : ) = updated_map.colors  ( r , c , : );
      map_normals ( i , : ) = updated_map.normals ( r , c , : );
      map_ccounts ( i     ) = updated_map.ccounts ( i     );
      map_times   ( i     ) = updated_map.times   ( i     );
    end

    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   
