function [dis_true] = dis_cal(posi_1,posi_2)

dis_true = sqrt((posi_1- posi_2)' * (posi_1- posi_2));

end