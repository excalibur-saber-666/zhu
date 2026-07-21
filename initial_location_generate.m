clear;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------初始位置生成----------%
%%
uav_num = 12;
[x,y,z]=sampling(0,600,0,600,100,800,uav_num,80);
% load ('posi_e_all.dat');load ('posi_n_all.dat');load ('posi_u_all.dat');

figure_num = 1;
figure(figure_num);
plot3(x,y,z,'o');grid;

save posi_e_all.dat x -ASCII;
save posi_n_all.dat y -ASCII;
save posi_u_all.dat z -ASCII;