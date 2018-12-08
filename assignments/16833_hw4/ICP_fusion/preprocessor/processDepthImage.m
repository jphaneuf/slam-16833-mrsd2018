% take in a depth image and convert to XYZ
% ICP fusion takes in format like so:
% {1x1 pointCloud}    {480x640x3 double}
% {1x1 pointCloud}    {480x640x3 double}
% of pointCloud (from XYZ image) and RGB image
data_in_path = '../geovizdata2/';
data_out_path = strcat(data_in_path,'processed/');
data_dir_obj = dir(strcat(data_in_path,'*.mat'));
load(strcat(data_in_path,'processed/cam_param.mat')); %need h, w, d of image
data_file = strcat(data_in_path,'ptClouds.mat');
%h = 1920;
%w = 1080;
%d = 3;
%DEPTH_CUTOFF = 3;

input_seq_struct = load(data_file);
input_seq = input_seq_struct.ptClouds;

for i = 1:size(input_seq,2)
  pcl = input_seq{i};
  locs = pcl.Location; %extract point vector from point cloud
  %Plane fit 1
  [plane_locs, plane_indices,outliderIndicies] = pcfitplane(pcl, 20);
  locs(plane_indices,:) = nan;
  %plane fit 2
  pcl2 = pointCloud(locs);
  [plane_locs, plane_indices,outliderIndicies] = pcfitplane(pcl2, 20);
  locs(plane_indices,:) = nan;

  %%Filtering
  %[filt_locs inl_ind out_ind] = pcdenoise(pointCloud(locs),'Threshold',0.01,'NumNeighbors',20);
  %locs(out_ind,:) = nan;
  locs = medfilt3(locs,[3 3 3]);

  %Reshape locs and colors
  locs = reshape(locs,h,w,d);
  colors = pcl.Color;
  colors = reshape(colors,h,w,d);
  %colors(plane_indices,:) = repmat([255 0 0], size(plane_indices,1),1);

  %Compute surface normals
  compute_normals_pcl = pointCloud(locs);
  normals = pcnormals(compute_normals_pcl);
  imshow(normals);
   
  %Use normals representation to crop image
  disp('select two points on corners of bounding box around object')
  [x y] = ginput(2);
  x_min = min(x);
  x_max = max(x);
  y_min = min(y);
  y_max = max(y);
  normals(:,1:x_min,:) = nan;
  normals(:,x_max:end,:) = nan;
  normals(1:y_min,:,:) = nan;
  normals(y_max:end,:,:) = nan;
  locs(:,1:x_min,:) = nan;
  locs(:,x_max:end,:) = nan;
  locs(1:y_min,:,:) = nan;
  locs(y_max:end,:,:) = nan;
  new_pcl = pointCloud(locs);
  new_pcl.Color = colors;


  %normals = reshape (normals, h, w, d);
  seq{i,1} = new_pcl;
  seq{i,2} = normals;
  %showPointCloud(seq{i,1})
  %pause
end

save(strcat(data_out_path,'pre.mat'),'seq');
