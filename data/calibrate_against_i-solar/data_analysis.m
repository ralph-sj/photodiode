% analyse 2015-10-27
clear
close all
[Vpd,t_Vpd,I,t_I] = importfile('data_2015-10-27.csv', 1);
createfigure(Vpd,I);