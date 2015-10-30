% analyse 2015-10-27
clear
close all
[status,temperature,acceleration_y,acceleration_x,humidity,pressure,v_pd,v_cc,id,acceleration_z,timestamp] = importfile('../data/stream_G2x5VRRgqguX5nmAgnoX.csv', 2);
% createfigure(Vpd,I);

figure
plot(v_pd, '.-');