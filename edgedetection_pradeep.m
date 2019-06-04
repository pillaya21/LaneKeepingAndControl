clear
clc
clc
clear all
close all
tic;
i=0;
mov=VideoReader('sample_video.mp4');
while hasFrame(mov)
    i=i+1;
    I=readFrame(mov);
    I=imresize(I,0.25);
    shape = size(I);
    target_x=0.4; target_y=0.6*shape(1);
    GS=rgb2gray(I);
    E1=edge(GS,'canny',[0.25,0.35]);
    % figure(2);
    % imshow(E1);
    a=[shape(2)*target_x,shape(2)*(1-target_x),shape(2),   0    ];
    b=[target_y,target_y,shape(1),shape(1)];
    bw=roipoly(I,a,b);
    E=(E1(:,:,1)&bw);
    [H,theta,rho]=hough(E);
    P = houghpeaks(H,6,'threshold',8);
    lines = houghlines(E,theta,rho,P,'FillGap',60,'MinLength',10);
    figure(1);
    imshow(I);
    hold on
    max_len = 0;p=1;q=1;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       theta=lines(k).theta;
       if theta>=0
           leftline(p,:)=[lines(k).point1, lines(k).point2];
           p=p+1;
       else
           rightline(q,:)=[lines(k).point1, lines(k).point2];
           q=q+1;
       end
%       Determine the endpoints of the longest line segment
%        len = norm(lines(k).point1 - lines(k).point2);
%        if ( len > max_len)
%           max_len = len;
%           xy_long = xy;
%        end
    end
    Lxpoints=leftline(:,1);leftline(:,3);
    Lypoints=leftline(:,2);leftline(:,4);
    Rxpoints=rightline(:,1);rightline(:,3);
    Rypoints=rightline(:,2);rightline(:,4);
    L=polyfit(Lxpoints,Lypoints,1);
    R=polyfit(Rxpoints,Rypoints,1);
    plot(Lxpoints,(Lxpoints*L(1)+L(2)),'LineWidth',4,'Color','red')
    plot(Rxpoints,(Rxpoints*R(1)+R(2)),'LineWidth',4,'Color','Green')
    %hold on
end

