%% rotationMatrix
% figure
clc
clear all
close all
tmpRot=dlmread('rotMatLog.txt');

%% acceleration
tmp=dlmread('accLog.txt');
acc(:, 1)=tmp(:, 1);
acc(:, 2)=-tmp(:, 3);
acc(:, 3)=tmp(:, 2);

% plot(acc), legend('x', 'y', 'z')

%% gps

tmp=dlmread('gpsLog.txt');
% gps(:, 1)=tmp(:, 1);
% gps(:, 2)=-tmp(:, 3);
% gps(:, 3)=tmp(:, 2);
gps=tmp;
%%
% figure
% hv1=plot3(rand(), rand(), rand()); hold on;
% hv2=plot3(rand(), rand(), rand());
% hv3=plot3(rand(), rand(), rand());
% legend('v1', 'v2', 'v3');
% 
% axis([-1 1 -1 1 -1 1]*1.5)


for i=1:length(tmpRot)
    Rtmp=vec2mat(tmpRot(i, :), 3);
    
    %modify R

    R=Rtmp;
    R(:, 3)=-R(:, 3);
    
    v1=[0 0 0;  R(1:3, 1)'];
    v2=[0 0 0;  R(1:3, 2)'];
    v3=[0 0 0;  R(1:3, 3)'];
    
%     set(hv1, 'xdata', v1(:, 1), 'ydata', v1(:, 2)', 'zdata', v1(:, 3));
%     set(hv2, 'xdata', v2(:, 1), 'ydata', v2(:, 2)', 'zdata', v2(:, 3));
%     set(hv3, 'xdata', v3(:, 1), 'ydata', v3(:, 2)', 'zdata', v3(:, 3));
%     drawnow;
%     pause(1e-6);
    
    i
    acc2(i, 1:3)=(R*acc(i, :)')';
    gps2(i, 1:3)=(R*gps(i, :)')';
end


%%
% figure
% plot(acc), legend('x', 'y', 'z'), grid on
% title('acc')
% 
% figure
% plot(acc2), legend('x', 'y', 'z'), grid on
% title('acc2')

figure
plot3(gps(:, 1), gps(:, 2), gps(:, 3)), xlabel('x'), ylabel('y'), zlabel('z'), hold on

plot3(gps2(:, 1), gps2(:, 2), gps2(:, 3)), xlabel('x'), ylabel('y'), zlabel('z')

