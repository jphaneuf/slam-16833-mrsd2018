function [pc] = pcNans2Zero(pc)
  locs = pc.Location;
  %[h w d] = size(locs);
  colors = pc.Color;
  locs(isnan(locs)) = 0;

  pc = pointCloud(locs);
  pc.Color = colors;
end
