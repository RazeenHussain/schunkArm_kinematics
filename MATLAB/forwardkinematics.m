% FORWARD KINEMATICS CODE FOR SCHUNK POWERBALL LWA 4P

% INITIALIZATION
close all;
clear all;
clc;
syms theta1 theta2 theta3 theta4 theta5 theta6;

% DH PARAMETERS ASSIGNMENT FOR THE LINKS
L1 = Link ([      0,   0.205,      0,    -pi/2,    0]);
L2 = Link ([  -pi/2,       0,  0.350,       pi,    0]);
L3 = Link ([  -pi/2,       0,      0,    -pi/2,    0]);
L4 = Link ([      0,   0.305,      0,     pi/2,    0]);
L5 = Link ([      0,       0,      0,    -pi/2,    0]);
L6 = Link ([      0,   0.075,      0,        0,    0]);

% SETTING LIMITS FOR JOINT ANGLES
L1.qlim=[-170 170]*pi/180;
L2.qlim=[-110 110]*pi/180;
L3.qlim=[-155 155]*pi/180;
L4.qlim=[-170 170]*pi/180;
L5.qlim=[-140 140]*pi/180;
L6.qlim=[-170 170]*pi/180;

% HOMOGENEOUS TRANSFORMATION MATRIX FOR THE LINKS
T1 = L1.A(theta1);
T2 = L2.A(theta2);
T3 = L3.A(theta3);
T4 = L4.A(theta4);
T5 = L5.A(theta5);
T6 = L6.A(theta6);

% FORWARD KINEMATICS MATRIX FOR THE END EFFECTOR
forward = T1 * T2 * T3 * T4 * T5 * T6;
%disp(vpa(forward));

% ROBOT PLOT AT ZERO JOINT ANGLES
powerball = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'POWERBALL');
powerball.plot([0 0 0 0 0 0]);

powerball.fkine([0 0 0 0 0 0])

powerball.fkine([0 0.4 0 -0.5 0 1])


% INVERSE AND FORWARD CHECK
%final = powerball.fkine([0 pi/6 0 0 0 0]);
%disp(final);
%q2 = powerball.ikine(final);
%disp(vpa(q2));
%temp = powerball.fkine(q2);
%disp(vpa(temp));

% SIMULATION OF ROBOT USING JOINT ANGLES
%from = [0 0 0 0 0 0];
%to = [0 0.4 0 -0.5 0 1];
%move = jtraj(from, to, 10);
%powerball.plot(move);

% SIMULATION USING WRIST COORDINATES
start = transl(0, 0, 0); 
destination = transl(0.4, 0, 0.6);	
inverse = ctraj(start, destination, 50);
q=powerball.ikine(inverse);
powerball.plot(q);



%B1 = transform (theta1,   0.205,      0,    -pi/2);
%B2 = transform (theta2-pi/2,       0,  0.350,       pi);
%B3 = transform (theta3-pi/2,       0,      0,    -pi/2);
%B4 = transform (theta4,   0.305,      0,     pi/2);
%B5 = transform (theta5,       0,      0,    -pi/2);
%B6 = transform (theta6,   0.075,      0,        0);
%B=B1*B2*B3*B4*B5*B6;
%disp(vpa(B1));
%disp(vpa(B2));
%disp(vpa(B3));
%disp(vpa(B4));
%disp(vpa(B5));
%disp(vpa(B6));
%disp(vpa(B));
