function [ds_pointcloud, ds_normals] = downsampleData(pointcloud, normals, ds_ratio)
    
		%ds_ratio = 1/ds_ratio;
    %==== Downsample both pointcloud and normals ====
    %ds_points = pointcloud.Location(1:ds_ratio:end, 1:ds_ratio:end, :);
    %ds_colors = pointcloud.Color(1:ds_ratio:end, 1:ds_ratio:end, :);
		%ds_points = imresize(pointcloud.Location,ds_ratio);
		%ds_colors = imresize(pointcloud.Color,ds_ratio);

    %ds_pointcloud = pointCloud(ds_points, 'Color', ds_colors);
    %ds_normals = normals(1:ds_ratio:end, 1:ds_ratio:end, :);
    %ds_normals = imresize(normals, ds_ratio);
		ds_pointcloud = pcdownsample(pointcloud, 'gridaverage', ds_ratio);
		temp_pc = pointCloud(normals);
		temp_pc = pcdownsample(temp_pc, 'gridaverage', ds_ratio)
		ds_normals = temp_pc.Location;
    
end
    
