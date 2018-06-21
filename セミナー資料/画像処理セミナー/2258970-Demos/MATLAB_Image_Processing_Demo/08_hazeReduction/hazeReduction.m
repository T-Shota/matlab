%% –¶‚Ìœ‹
% 
%% ‰æ‘œ“Ç‚İ‚İ
A = imread('foggysf2.jpg');
figure, imshow(A);

%% –¶‚Ìœ‹
B = imreducehaze(A, 0.9, 'method', 'approxdcp');
figure, imshow(B);

%% •À‚×‚Ä•\¦
figure, imshowpair(A, B, 'montage')