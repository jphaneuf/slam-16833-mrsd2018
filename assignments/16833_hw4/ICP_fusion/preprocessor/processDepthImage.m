% take in a depth image and convert to XYZ
% ICP fusion takes in format like so:
% {1x1 pointCloud}    {480x640x3 double}
% {1x1 pointCloud}    {480x640x3 double}
% of pointCloud (from XYZ image) and RGB image
data_in_path = '../geovizdata2/';
data_out_path = strcat(data_in_path,'processed/');
data_dir_obj = dir(strcat(data_in_path,'*.mat'));
load(strcat(data_in_path,'processed/cam_param.mat'));
data_file = strcat(data_in_path,'ptClouds.mat');
%h = 1920;
%w = 1080;
%d = 3;

input_seq_struct = load(data_file);
input_seq = input_seq_struct.ptClouds;
for i = 1:size(input_seq,2)
 pcl = input_seq{i};
 [filt_locs inl_ind out_ind] = pcdenoise(pcl,'Threshold',0.1);
 locs = pcl.Location;
 locs(out_ind) = nan;
 locs = reshape(locs,h,w,d);
 colors = pcl.Color;
 colors = reshape(colors,h,w,d);
 new_pcl = pointCloud(locs);
 new_pcl.Color = colors;
 normals = pcnormals(new_pcl);
 %normals = reshape (normals, h, w, d);
 seq{i,1} = new_pcl;
 seq{i,2} = normals;
end

save(strcat(data_out_path,'pre.mat'),'seq');
