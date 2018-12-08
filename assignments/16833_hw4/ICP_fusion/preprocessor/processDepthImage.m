% take in a depth image and convert to XYZ
% ICP fusion takes in format like so:
% {1x1 pointCloud}    {480x640x3 double}
% {1x1 pointCloud}    {480x640x3 double}
% of pointCloud (from XYZ image) and RGB image
%pcnormals(pcl)
data_in_path = '../geovizdata1/';
data_out_path = '../geovizdata1/processed/';
data_dir_obj = dir(strcat(data_in_path,'*.mat'));
load('../geovizdata1/processed/cam_param.mat');
%hacks
%h = 1920;
%w = 1080;
%d = 3;

seq = {};
for i = 1:length(data_dir_obj)
 file_path = strcat(data_in_path,data_dir_obj(i).name);
 pcl=load(file_path);
 pcl = pcl.ptCloud;
 locs = pcl.Location;
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
