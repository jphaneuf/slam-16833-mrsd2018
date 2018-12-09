% take in a depth image and convert to XYZ
% ICP fusion takes in format like so:
% {1x1 pointCloud}    {480x640x3 double}
% {1x1 pointCloud}    {480x640x3 double}
% of pointCloud (from XYZ image) and RGB image
function processDepthImage(i)
  data_in_path = '../geovizdata5/';
  data_out_path = strcat(data_in_path,'processed/');
  data_dir_obj = dir(strcat(data_in_path,'*.mat'));
  load(strcat(data_in_path,'processed/cam_param.mat')); %need h, w, d of image
  data_file = strcat(data_in_path,'ptClouds.mat');

  fx = cam_param(1);
  fy = cam_param(2);
  cx = cam_param(3);
  cy = cam_param(4);
  K = [ fx 0 cx ; 0 fy cy ; 0 0 1 ];


  input_seq_struct = load(data_file);
  input_seq = input_seq_struct.ptClouds;

  pcl = input_seq{i};
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
  [proj_points proj_colors] = projectPly2RGBD(pcl, cam_param, h, w);
  proj_points = reshape(proj_points,[],3);
  depth = vecnorm(proj_points')';
  depth = reshape(depth,h,w,1);
  scale = max(max(depth));
  depth_filt = medfilt2(depth,[21 21]);
  depth(depth==0) = nan;
  %depth_filt = bfilt2(double(depth_filt)/scale,rgb2gray(proj_colors),9,1.2,0.25); 
  %depth_filt = scale*double(depth_filt)/255;

  output = zeros(h,w,3);
  for y = 1:h
    for x = 1:w
      p = [x; y; 1];	
      %back = depth(y,x)*inv(K)*p;
      ray = inv(K)*p;
      ray = ray/norm(ray);
      %ray = depth(y,x) * ray;
      ray = depth_filt(y,x) * ray;
      output(y,x,1) = ray(1);
      output(y,x,2) = ray(2);
      output(y,x,3) = ray(3);
    end
  end
  pcl_mf = pointCloud(output); %median filtered
  pcl_mf.Color = uint8(proj_colors);

  locs = pcl_mf.Location;
  while 1
    temp_normals = pcnormals(pointCloud(locs));
    imshow(temp_normals(:,:,1),[]);
    myhandsaresmalliknow = imfreehand()
    roi_mask = ~myhandsaresmalliknow.createMask();
    %locs = applyMaskToRGB(locs, roi_mask, nan);
    imshow(applyMaskToRGB(locs, roi_mask, nan),[]);
    usr_input = input('c to commit edit and keep editing, y to quit' ,'s');
    if usr_input == 'y'
      break;
    elseif usr_input == 'c'
      locs = applyMaskToRGB(locs, roi_mask, nan);
    end
  end

  locs = reshape(locs,[],3);
  [filt_locs inl_ind out_ind] = pcdenoise(pointCloud(locs),'Threshold',0.5,'NumNeighbors',10);
  locs(out_ind,:) = nan;
  locs = reshape(locs,h,w,d);
  %{
  %}

  pcl_out = pointCloud(locs);
  pcl_out.Color = uint8(proj_colors);
 
  normals = pcnormals(pcl_out);

  output_file = strcat(data_out_path,'pre.mat');
  if isfile(output_file)
    load(output_file);
  end
  seq{i,1} = pcl_out;
  seq{i,2} = normals;
  save(strcat(data_out_path,'pre.mat'),'seq');
  %showPointCloud(seq{i,1})
  %pause
  %save(strcat(data_out_path,'pre.mat'),'seq');
end
