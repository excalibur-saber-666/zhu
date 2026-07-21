function [x,y,z]=sampling(lowx,upx,lowy,upy,lowz,upz,m,n)
%该函数用来产生不重复的随机整数矩阵
%low—随机整数下界；up—随机整数上界；m—坐标点个数；n—各个坐标点之间的最小距离
%编写函数时的测试数据
% lowx=0;upx=700; lowy=0;upy=700; lowz=0;upz=800;m=20;n=10;

s=[];%矩阵s，第一行存储x，第二行存储y，第三行存储z
i=1;
c=1; %控制循环次数
while c<5000
    tempx=randi([lowx,upx],1);
    tempy=randi([lowy,upy],1);
    tempz=randi([lowz,upz],1);
    %如果矩阵s为空矩阵，则存储第一个随机数值
    if(isempty(s)==1)
        s(1,i)=tempx;
        s(2,i)=tempy;
        s(3,i)=tempz;
        i=i+1;
        c=c+1;
    end
    %如果矩阵s不为空矩阵，求出各个坐标之间的距离矩阵l，将与所有点之间距离都大于10的坐标存储
    if(isempty(s)==0)
        for j=1:i-1
            l(1,j)=sqrt(((tempx-s(1,j))^2)+((tempy-s(2,j))^2)+((tempz-s(3,j))^2));
            j=j+1;
        end
        if(isempty(find(l<n,1))==1)  %或size(find(l<n),1)==0 表示矩阵l中小于n的个数等于0
        s(1,i)=tempx;
        s(2,i)=tempy;
        s(3,i)=tempz;
        i=i+1; 
        end
        c=c+1;
    end
    if(i>=m+1)
        break;
    end
end
x=s(1,:);
y=s(2,:);
z=s(3,:);
