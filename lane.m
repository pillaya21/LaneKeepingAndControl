%edge detection
pic = imread('test.jpg');
shape = size(pic);
gray_pic = rgb2gray(pic);
edge_pic = edge(gray_pic,'canny',[0.25,0.36]);
imshow(edge_pic);
% %region masking
% target_x=0.4; target_y=0.6*shape(1);
% a=[shape(2)*target_x,shape(2)*(1-target_x),shape(2),   0    ];
% b=[target_y,target_y,shape(1),shape(1)];
% bw=roipoly(pic,a,b);
% BW=(edge_pic(:,:,1)&bw);
% imshow(BW);
[H,T,R] = hough(BW);
P=houghpeaks(H,3);

lines = houghlines(BW,T,R,P,'FillGap',4,'MinLength',5);
imagesc(pic);
hold on;
for i= 1:length(lines)
  plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2),lines(i).point2(2)],'LineWidth',2,'Color','red');
end

