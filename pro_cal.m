function [Ck,Uk_mix] = pro_cal(P,Uk)
 
 Ck = zeros(2,1);
 Uk_mix = zeros(2,2);
 
 for i =1:2
     for j=1:2
         Ck(i,1) = Ck(i,1) + P(j,i)*Uk(j,1);  %Uk为每种状态的概率，P为由这种状态向另一种状态转移的概率
     end
 end
 
 for i =1:2
     for j = 1:2
         Uk_mix(i,j) = P(i,j)*Uk(i,1)/Ck(j,1);
     end
 end
