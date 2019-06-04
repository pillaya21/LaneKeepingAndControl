clc
clear
close all
i=1;K1=0.08;K2=1;
Cam=webcam(1);
Cam.Resolution='1920x1080';
a=arduino('COM5','UNO','Libraries','servo');
V=servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
S=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
writePosition(V, 0.5);
writePosition(S, 0.5);
pause(0.05);
Left =[];Right =[];L=[];R=[];
vel=0.559;
%KALMAN FILTER FOR STEERING
PS=0;QS=0.003;RS=0.003;Steer=0.37;
% Errors for Kalman gain for Left and Right Lanes;
% EC=zeros(4,4);
% EQ=1000*[0.05,0.0,0.0,0.0;
%     0,0.09,00,00;
%     0.0,0.0,0.08,0.0;
%     0,00,0,0.0001;];
% ER=100*[29,0.0,0.0,0.0;
%     0,0.01,00,00;
%     0.0,0,29,0.0;
%     0,00,0,0.02;];

% %Kalman gains for Middle Lane;
% EC=zeros(2,2);
% EQ=[1,0;0,1];ER=[1,0;0,1];
while (1)
    
    p=snapshot(Cam); p=imresize(p,1);
    shape=size(p);
    p1=rgb2gray(p);
    %detect edges
    BW=edge(p1,'canny',[0.31,0.36]);
    %Region Masking
    a=[shape(2)*0.5/6,shape(2)*5.5/6,shape(2),0];
    b=[shape(1)*3/7,shape(1)*3/7,shape(1),shape(1)];
    
    %     target_x=0.3; target_y=0.4*shape(1); oset=0.9;
    %     a=[0,shape(2)*target_x,shape(2)*(1-target_x),shape(2),shape(2),0];% x coordinates of trapezoid
    %     b=[shape(1)*oset,target_y,target_y,shape(1)*oset,shape(1),shape(1)]; % y coordinates of trepezoid
    
    bw=roipoly(p,a,b);
    BW=(BW(:,:)&bw);
    sh=size(BW);
    [H,theta,rho]=hough(BW,'Theta',-60:60); %Transform Display
    
    P = houghpeaks(H,4,'threshold',0.3*max(max((H))));
    %first number of peaks to identify Threshold- minimum value to be
    %considered a peak
    
    lines = houghlines(BW,theta,rho,P,'FillGap',20,'MinLength',15);
    %Minimum -lines shorter than that are omitted
    %When the distance between the line segments is less than the value
    %specified, the houghlines function merges the line segments into
    %a single line segment.
    
    figure(01)
%      imshow(p), hold on
    L1=[];R1=[];Ltheta=[];Rtheta=[];L2=[];R2=[];
    for k = 1:length(lines)
        xy(k,:) = [lines(k).point1,lines(k).point2];
        theta=lines(k).theta;
        if theta>=0
            L1=[L1;lines(k).point1;]; %points for left line
            L2=[L2;lines(k).point2;];
            Ltheta=[Ltheta;lines(k).theta];
        else
            R1=[R1;lines(k).point1;];
            R2=[R2;lines(k).point2;]; %points for right line
            Rtheta=[Rtheta;lines(k).theta];
        end
    end
    L=[L1;L2];
    R=[R1;R2];
    %     if length(Ltheta)>1
    %         Lsigma=std(Ltheta);
    %         Lbar=mean(Ltheta);
    %         Ldiff=abs(Lbar-Ltheta);
    %         CheckL=abs(Ldiff<Lsigma)
    %         L(CheckL,:)=[];
    %     end
    %     if length(Rtheta)>1
    %         Rsigma=std(Rtheta);
    %         Rbar=mean(Rtheta);
    %         Rdiff=abs(Rbar-Rtheta);
    %         RCheck=abs(Rdiff<Lsigma)
    %         L(RCheck,:)=[];
    %     end
    %     if i~=1
    %         X=[Left,Right]';
    %     end
    if ~isempty(L)
        Lx=L(:,1);
        Ly=L(:,2);
        Left=polyfit(L(:,1),L(:,2),1);
    else
        Lx=[];
        Ly=[];
    end
    if ~isempty(R)
        Rx=R(:,1);
        Ry=R(:,2);
        Right=polyfit(R(:,1),R(:,2),1);
    else
        Rx=[];
        Ry=[];
    end
    %     %         Kalman Filter for Left and Right
    %     if isempty(X)
    X=[Left,Right]';
    %     end
    %     X0=X;
    %     X1=[Left,Right]';
    %     EC=EC+EQ;
    %     K=EC/(EC+ER);
    %     X=X0+K*(X1-X0);
    %     EC=(eye(4)-K)*EC;
    %     Left=[X(1),X(2)];
    %     Right=[X(3),X(4)];
    %         Kalman Filter End Left and Right
    %     Lxx=min(Lx):max(Lx);
    %     Rxx=min(Rx):max(Rx);
    %     Mx(1)=0.5*(min(Lx)+max(Rx));
    %     Mx(2)=0.5*(max(Lx)+min(Rx));
    %     My(1)=0.5*(polyval(Left,min(Lx))+polyval(Right,max(Rx)));
    %     My(2)=0.5*(polyval(Left,max(Lx))+polyval(Right,min(Rx)));
    Yr=sh(1)*0.5;
    %end points of left line
    Lxn=(Yr*2-X(2))/X(1);
    Lxf=(Yr-X(2))/X(1);
    %end points of right line
    Rxn=(Yr*2-X(4))/X(3);
    Rxf=(Yr-X(4))/X(3);
    %Middle line points
    Mxn=0.5*(Lxn+Rxn);
    Mxf=0.5*(Lxf+Rxf);
    Myn=Yr*2;
    Myf=Yr;
    %     Middle=polyfit(Mx,My,1);
    % % %         Kalman gain for Middle Line
    %         if i~=1
    %             X0=X;
    %             X1=[Middle]';
    %             EC=EC+EQ;
    %             K=EC*pinv(EC+ER);
    %             X=X0+K*(X1-X0);
    %             EC=(eye(2)-K)*EC;
    %             Middle=X';
    %         end
    %     Mxx=min(Mx):0.1:max(Mx);
    plot(Lx,polyval(Left,Lx),'LineWidth',2,'Color','green')
    plot(Rx,polyval(Right,Rx),'LineWidth',2,'Color','green')
    plot([Mxn,Mxf],[Myn,Myf],'LineWidth',2,'Color','blue')
    Xr=sh(2)*0.5;
    %     Yr=sh(1)*0.5;
    %     m=Middle(1);
    plot([Xr,Xr],[0,Yr*2],'LineWidth',2,'Color','red');
    %Control Logic
    %     if (max(Mx)-min(Mx))==0
    %         angle=0;
    %     else
    %         angle=atan((max(Mx)-min(Mx))/(polyval(Middle,max(Mx))-polyval(Middle,min(Mx))));
    %     end
    angle = atan((Mxf-Mxn)/(Myf-Myn));
    %distance
    %     Xm=((Yr*0.5)-Middle(2))/Middle(1);
    %     Xl=((Yr*0.5)-Left(2))/Left(1);
    %     Xri=((Yr*0.5)-Right(2))/Right(1);
    %     D=(Xr-Xm)/(Xl-Xri);
    D=(Mxn-Xr)/(Lxn-Rxn);
    %Steer Angle
    Sp=Steer;
    K2=1.98;
    Steer=0.37+0.5*(K1*angle+K2*D);
    
    %yyyyyzzz Kalman Filter
    %prediction
    PS=PS+QS;
    %correction
    K=PS/(PS+RS);
    Steer=Sp+K*(Steer-Sp);
    PS=(1-K)*PS;
    
    if Steer>0.8
        Steer=0.8
    elseif Steer<0.2
        Steer=0.2
    else
        Steer=Steer;
    end
        writePosition(V,vel);
        Steer
        writePosition(S,Steer);
    drawnow
    i=i+1;
end