function helperDisplayImageMontage(cellArrayOfImages)
% Displays a montage of images. Images are resized to handle different
% image sizes.

thumbnails = [];
for i = 1:numel(cellArrayOfImages)
    img = imread(cellArrayOfImages{i});
    thumbnails = cat(4, thumbnails, imresize(img, [200 200]));
end

figure
montage(thumbnails)
