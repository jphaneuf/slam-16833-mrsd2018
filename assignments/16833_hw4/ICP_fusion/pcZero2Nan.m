function [pc] = pcZero2Nan(pc)
  locs = pc.Location;
  %[h w d] = size(locs);
  colors = pc.Color;
  locs(locs==0) = nan;

  pc = pointCloud(locs);
  pc.Color = colors;
end
