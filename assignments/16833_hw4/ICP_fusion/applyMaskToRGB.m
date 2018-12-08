function [img] = applyMaskToRGB(img,mask,value)
  r = img(:,:,1);
  g = img(:,:,2);
  b = img(:,:,3);
  r(mask) = value;
  g(mask) = value;
  b(mask) = value;
  img(:,:,1) = r;
  img(:,:,2) = g;
  img(:,:,3) = b;
end


