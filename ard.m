%arduino
clc
clear
a=arduino('COM5','UNO','Libraries','servo')
V=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
S=servo(a,'D8','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
vel=1
i=0;
writePosition(S,0.7);
pause(0.5)

writePosition(V,0.7);
pause(0.5);

writePosition(S,0.565);
% while (1)
%     i=i+1;
%      writePosition(S,0);
%  pause(0.5);
%  if(rem(i,2))
%      Ss=0.3;
%  else
%      Ss=0.8
%  end
%  writePosition(S,Ss);
%  pause(0.5);
% end
% %     %         Kalman Filter for Left and Right
%     if i==1
%         X=[Left,Right]';
%     end
%     X0=X;%prediction
%     X1=[Left,Right]';%measurement
%     EC=EC+EQ;%error covariance
%     K=EC/(EC+ER);%kalman gain
%     X=X0+K*(X1-X0);%correction
%     EC=(eye(4)-K)*EC;%error
%     Left=[X(1),X(2)];
%     Right=[X(3),X(4)];
%     %     Kalman Filter End Left and Right
