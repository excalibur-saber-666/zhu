clear;
close all;

randn('seed',150);

load('Trace_gps.dat');
load('Trace_graph.dat');
load('Trace_gps_non_iter.dat');
load('Trace_graph_non_iter.dat');

uav_num = 18;
[x,y,z]=sampling(0,200,0,100,100,300,uav_num,15);
load ('x.dat');load ('y.dat');load ('z.dat');

dis = zeros(uav_num,uav_num);
for i = 1: uav_num
    for j = 1:uav_num
        dis(j,i) = DistanceAB([x(1,j);y(1,j);z(1,j)],[x(1,i);y(1,i);z(1,i)]);
    end
end
save x.dat x -ASCII;
save y.dat y -ASCII;
save z.dat z -ASCII;

num_in = zeros(uav_num,1);
for i = 1:uav_num
    for j =1:uav_num
        if dis(i,j) <100
            num_in(i,1) = num_in(i,1) + 1;
        end
    end
end

posi_L_all = [x(1,2) x(1,4) x(1,16); y(1,2) y(1,4) y(1,16); z(1,2) z(1,4) z(1,16)];
posi_w_all = [];

for i = 1:(uav_num)
    if i == 4 || i == 2 || i == 16
%         disp('not wings')
    else
        posi_w_i = [x(1,i);y(1,i);z(1,i)];
        posi_w_all = [posi_w_all posi_w_i];
    end
end

figure_num = 0;
% figure_num = figure_num + 1;
% figure(figure_num);
% plot3(posi_L_all(1,1),posi_L_all(2,1),posi_L_all(3,1),'o');hold on
% plot3(posi_L_all(1,2),posi_L_all(2,2),posi_L_all(3,2),'o');hold on
% plot3(posi_L_all(1,3),posi_L_all(2,3),posi_L_all(3,3),'o');hold on
% plot3(posi_w_all(1,1),posi_w_all(2,1),posi_w_all(3,1),'*');hold on
% plot3(posi_w_all(1,2),posi_w_all(2,2),posi_w_all(3,2),'*');hold on
% plot3(posi_w_all(1,3),posi_w_all(2,3),posi_w_all(3,3),'*');hold on
% plot3(posi_w_all(1,4),posi_w_all(2,4),posi_w_all(3,4),'*');hold on
% plot3(posi_w_all(1,5),posi_w_all(2,5),posi_w_all(3,5),'*');hold on
% plot3(posi_w_all(1,6),posi_w_all(2,6),posi_w_all(3,6),'*');hold on
% plot3(posi_w_all(1,7),posi_w_all(2,7),posi_w_all(3,7),'*');hold on
% plot3(posi_w_all(1,8),posi_w_all(2,8),posi_w_all(3,8),'*');hold on
% plot3(posi_w_all(1,9),posi_w_all(2,9),posi_w_all(3,9),'*');hold on
% plot3(posi_w_all(1,10),posi_w_all(2,10),posi_w_all(3,10),'*');hold on
% plot3(posi_w_all(1,11),posi_w_all(2,11),posi_w_all(3,11),'*');hold on
% plot3(posi_w_all(1,12),posi_w_all(2,12),posi_w_all(3,12),'*');hold on
% plot3(posi_w_all(1,13),posi_w_all(2,13),posi_w_all(3,13),'*');hold on
% plot3(posi_w_all(1,14),posi_w_all(2,14),posi_w_all(3,14),'*');hold on
% plot3(posi_w_all(1,15),posi_w_all(2,15),posi_w_all(3,15),'*');hold on
% xlabel('东向/m');ylabel('北向/m');zlabel('天向/m');
% grid;

figure_num = figure_num + 1;
figure(figure_num);
plot3(posi_L_all(1,:),posi_L_all(2,:),posi_L_all(3,:),'o');hold on
plot3(posi_w_all(1,:),posi_w_all(2,:),posi_w_all(3,:),'*');hold on
grid;

figure_num = figure_num + 1;
figure(figure_num);
subplot(3,1,1);plot(Trace_graph(:,1),Trace_graph(:,2) - posi_w_all(1,1),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,2) - posi_w_all(1,1),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,2) - posi_w_all(1,1),'g');

subplot(3,1,2);plot(Trace_graph(:,1),Trace_graph(:,3) - posi_w_all(2,1),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,3) - posi_w_all(2,1),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,3) - posi_w_all(2,1),'g');

subplot(3,1,3);plot(Trace_graph(:,1),Trace_graph(:,4) - posi_w_all(3,1),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,4) - posi_w_all(3,1),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,4) - posi_w_all(3,1),'g');
grid;

figure_num = figure_num + 1;
figure(figure_num);
subplot(3,1,1);plot(Trace_graph(:,1),Trace_graph(:,26) - posi_w_all(1,9),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,26) - posi_w_all(1,9),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,26) - posi_w_all(1,9),'g');
subplot(3,1,2);plot(Trace_graph(:,1),Trace_graph(:,27) - posi_w_all(2,9),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,27) - posi_w_all(2,9),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,27) - posi_w_all(2,9),'g');
subplot(3,1,3);plot(Trace_graph(:,1),Trace_graph(:,28) - posi_w_all(3,9),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,28) - posi_w_all(3,9),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,28) - posi_w_all(3,9),'g');
grid;

figure_num = figure_num + 1;
figure(figure_num);
subplot(3,1,1);plot(Trace_graph(:,1),Trace_graph(:,32) - posi_w_all(1,11),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,32) - posi_w_all(1,11),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,32) - posi_w_all(1,11),'g');
subplot(3,1,2);plot(Trace_graph(:,1),Trace_graph(:,33) - posi_w_all(2,11),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,33) - posi_w_all(2,11),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,33) - posi_w_all(2,11),'g');
subplot(3,1,3);plot(Trace_graph(:,1),Trace_graph(:,34) - posi_w_all(3,11),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,34) - posi_w_all(3,11),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,34) - posi_w_all(3,11),'g');
grid;

figure_num = figure_num + 1;
figure(figure_num);
subplot(3,1,1);plot(Trace_graph(:,1),Trace_graph(:,35) - posi_w_all(1,12),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,35) - posi_w_all(1,12),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,35) - posi_w_all(1,12),'g');
subplot(3,1,2);plot(Trace_graph(:,1),Trace_graph(:,36) - posi_w_all(2,12),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,36) - posi_w_all(2,12),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,36) - posi_w_all(2,12),'g');
subplot(3,1,3);plot(Trace_graph(:,1),Trace_graph(:,37) - posi_w_all(3,12),'r');hold on
plot(Trace_gps(:,1),Trace_gps(:,37) - posi_w_all(3,12),'b');hold on;
plot(Trace_gps(:,1),Trace_graph_non_iter(:,37) - posi_w_all(3,12),'g');
grid;

sinserr_1 = [Trace_graph(:,2) - posi_w_all(1,1), Trace_graph(:,3) - posi_w_all(2,1),Trace_graph(:,4) - posi_w_all(3,1)];
var_e = sqrt(sum(sinserr_1(:,1).^2)/(length(sinserr_1(:,1))-1));
var_n = sqrt(sum(sinserr_1(:,2).^2)/(length(sinserr_1(:,1))-1));
var_u = sqrt(sum(sinserr_1(:,3).^2)/(length(sinserr_1(:,1))-1));

sinserr_2 = [Trace_graph(:,38) - posi_w_all(1,13), Trace_graph(:,39) - posi_w_all(2,13),Trace_graph(:,40) - posi_w_all(3,13)];
var_e_2 = sqrt(sum(sinserr_2(:,1).^2)/(length(sinserr_2(:,1))-1));
var_n_2 = sqrt(sum(sinserr_2(:,2).^2)/(length(sinserr_2(:,1))-1));
var_u_2 = sqrt(sum(sinserr_2(:,3).^2)/(length(sinserr_2(:,1))-1));

sinserr_3 = [Trace_graph(:,32) - posi_w_all(1,11), Trace_graph(:,33) - posi_w_all(2,11),Trace_graph(:,34) - posi_w_all(3,11)];
var_e_3 = sqrt(sum(sinserr_3(:,1).^2)/(length(sinserr_2(:,1))-1));
var_n_3 = sqrt(sum(sinserr_3(:,2).^2)/(length(sinserr_2(:,1))-1));
var_u_3 = sqrt(sum(sinserr_3(:,3).^2)/(length(sinserr_2(:,1))-1));

sinserr_4 = [Trace_graph(:,35) - posi_w_all(1,12), Trace_graph(:,36) - posi_w_all(2,12),Trace_graph(:,37) - posi_w_all(3,12)];
var_e_4 = sqrt(sum(sinserr_4(:,1).^2)/(length(sinserr_2(:,1))-1));
var_n_4 = sqrt(sum(sinserr_4(:,2).^2)/(length(sinserr_2(:,1))-1));
var_u_4 = sqrt(sum(sinserr_4(:,3).^2)/(length(sinserr_2(:,1))-1));

% sinserr_1_gps = [Trace_gps(:,2) - posi_w_all(1,1), Trace_gps(:,3) - posi_w_all(2,1),Trace_gps(:,4) - posi_w_all(3,1)];
% var_e_gps = sqrt(sum(sinserr_1_gps(:,1).^2)/(length(sinserr_1_gps(:,1))-1));
% var_n_gps = sqrt(sum(sinserr_1_gps(:,2).^2)/(length(sinserr_1_gps(:,1))-1));
% var_u_gps = sqrt(sum(sinserr_1_gps(:,3).^2)/(length(sinserr_1_gps(:,1))-1));
% 
% sinserr_2_gps = [Trace_gps(:,20) - posi_w_all(1,7), Trace_gps(:,21) - posi_w_all(2,7),Trace_gps(:,22) - posi_w_all(3,7)];
% var_e_2_gps = sqrt(sum(sinserr_2_gps(:,1).^2)/(length(sinserr_2_gps(:,1))-1));
% var_n_2_gps = sqrt(sum(sinserr_2_gps(:,2).^2)/(length(sinserr_2_gps(:,1))-1));
% var_u_2_gps = sqrt(sum(sinserr_2_gps(:,3).^2)/(length(sinserr_2_gps(:,1))-1));
% 
% sinserr_3_gps = [Trace_gps(:,32) - posi_w_all(1,11), Trace_gps(:,33) - posi_w_all(2,11),Trace_gps(:,34) - posi_w_all(3,11)];
% var_e_3_gps = sqrt(sum(sinserr_3_gps(:,1).^2)/(length(sinserr_2_gps(:,1))-1));
% var_n_3_gps = sqrt(sum(sinserr_3_gps(:,2).^2)/(length(sinserr_2_gps(:,1))-1));
% var_u_3_gps = sqrt(sum(sinserr_3_gps(:,3).^2)/(length(sinserr_2_gps(:,1))-1));
% 
% sinserr_4_gps = [Trace_gps(:,35) - posi_w_all(1,12), Trace_gps(:,36) - posi_w_all(2,12),Trace_gps(:,37) - posi_w_all(3,12)];
% var_e_4_gps = sqrt(sum(sinserr_4_gps(:,1).^2)/(length(sinserr_2_gps(:,1))-1));
% var_n_4_gps = sqrt(sum(sinserr_4_gps(:,2).^2)/(length(sinserr_2_gps(:,1))-1));
% var_u_4_gps = sqrt(sum(sinserr_4_gps(:,3).^2)/(length(sinserr_2_gps(:,1))-1));

sinserr_1_gps = [Trace_graph_non_iter(:,2) - posi_w_all(1,1), Trace_graph_non_iter(:,3) - posi_w_all(2,1),Trace_graph_non_iter(:,4) - posi_w_all(3,1)];
var_e_gps = sqrt(sum(sinserr_1_gps(:,1).^2)/(length(sinserr_1_gps(:,1))-1));
var_n_gps = sqrt(sum(sinserr_1_gps(:,2).^2)/(length(sinserr_1_gps(:,1))-1));
var_u_gps = sqrt(sum(sinserr_1_gps(:,3).^2)/(length(sinserr_1_gps(:,1))-1));

sinserr_2_gps = [Trace_graph_non_iter(:,38) - posi_w_all(1,13), Trace_graph_non_iter(:,39) - posi_w_all(2,13),Trace_graph_non_iter(:,40) - posi_w_all(3,13)];
var_e_2_gps = sqrt(sum(sinserr_2_gps(:,1).^2)/(length(sinserr_2_gps(:,1))-1));
var_n_2_gps = sqrt(sum(sinserr_2_gps(:,2).^2)/(length(sinserr_2_gps(:,1))-1));
var_u_2_gps = sqrt(sum(sinserr_2_gps(:,3).^2)/(length(sinserr_2_gps(:,1))-1));

sinserr_3_gps = [Trace_graph_non_iter(:,32) - posi_w_all(1,11), Trace_graph_non_iter(:,33) - posi_w_all(2,11),Trace_graph_non_iter(:,34) - posi_w_all(3,11)];
var_e_3_gps = sqrt(sum(sinserr_3_gps(:,1).^2)/(length(sinserr_2_gps(:,1))-1));
var_n_3_gps = sqrt(sum(sinserr_3_gps(:,2).^2)/(length(sinserr_2_gps(:,1))-1));
var_u_3_gps = sqrt(sum(sinserr_3_gps(:,3).^2)/(length(sinserr_2_gps(:,1))-1));

sinserr_4_gps = [Trace_graph_non_iter(:,35) - posi_w_all(1,12), Trace_graph_non_iter(:,36) - posi_w_all(2,12),Trace_graph_non_iter(:,37) - posi_w_all(3,12)];
var_e_4_gps = sqrt(sum(sinserr_4_gps(:,1).^2)/(length(sinserr_2_gps(:,1))-1));
var_n_4_gps = sqrt(sum(sinserr_4_gps(:,2).^2)/(length(sinserr_2_gps(:,1))-1));
var_u_4_gps = sqrt(sum(sinserr_4_gps(:,3).^2)/(length(sinserr_2_gps(:,1))-1));