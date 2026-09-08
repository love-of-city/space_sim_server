clc
clear
robot = importrobot('SARM.urdf');
robot.DataFormat = 'row';
q = [85.496	-83.219	52.257	30.962	-90.000	85.496];
q = q/180*pi;
f = [0 0];
p = [q f];
show(robot, p);
showdetails(robot);
