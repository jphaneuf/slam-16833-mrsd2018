% take in a depth image and convert to XYZ
% ICP fusion takes in format like so:
% {1x1 pointCloud}    {480x640x3 double}
% {1x1 pointCloud}    {480x640x3 double}
% of pointCloud (from XYZ image) and RGB image
%pcnormals(pcl)
data_in_path = '../geovizdata2/';
data_out_path = strcat(data_in_path,'processed/');
data_dir_obj = dir(strcat(data_in_path,'*.mat'));
load(strcat(data_in_path,'processed/cam_param.mat'));
%hacks
%h = 1920;
%w = 1080;
%d = 3;

seq = {};
for i = 1:length(data_dir_obj)
 file_path = strcat(data_in_path,data_dir_obj(i).name);
 pcl=load(file_path);
 pcl = pcl.ptCloud;
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
