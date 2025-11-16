%Initialize constants for the RCAM simulation
clear
clc
close all

% Simulation sample time
dt=1/50;
t_final=30;

% --- Initial Geodetic Position (Honolulu) ---
lon0 = deg2rad(-157.9251);   % Longitude (radians)
lat0 = deg2rad(21.3245);     % Latitude (radians)
h0   = 500;                  % Altitude (meters)


% Define constants
x0 = [85;        %approx 165 knots
      0;
      0;
      0;
      0;
      0;
      0;
      0.1;       %approx 5.73 deg
      0];
%x0 is a (9x1) matrix of states discussed so x0 is the intial state matrix

u = [0;
    -0.1;       %approx -5.73 deg
    0;
    0.4;       %recall minimum for throttles are 0.5*pi/180 = 0.0087
    0.4];
%similarly u us a initial input matrix of (5x1)

TF = 3*60;


% Define minimum/maximum control values (shown in your image)
u1min = -25*pi/180;
u1max =  25*pi/180;

u2min = -25*pi/180;
u2max =  10*pi/180;

u3min = -30*pi/180;
u3max =  30*pi/180;

u4min =  0;
u4max = 10*pi/180;

u5min =  0.5*pi/180;
u5max = 10*pi/180;

%%Run the model
out = sim('RCAMSimulation.slx');

% % Access state signals
% simX = out.simX;
% % or if in ans: simX = ans.simX;
% 
% % Access control inputs
% simU = out.simU;
% 
% % Time vector, if needed
% t = simX.Time;
% 
% % Unpack variables, exactly as in the image:
% u1 = simU.Data(:,1);
% u2 = simU.Data(:,2);
% u3 = simU.Data(:,3);
% u4 = simU.Data(:,4);
% u5 = simU.Data(:,5);
% 
% x1 = simX.Data(:,1);
% x2 = simX.Data(:,2);
% x3 = simX.Data(:,3);
% x4 = simX.Data(:,4);
% x5 = simX.Data(:,5);
% x6 = simX.Data(:,6);
% x7 = simX.Data(:,7);
% x8 = simX.Data(:,8);
% x9 = simX.Data(:,9);
% 
% %PLOTTING CONTROL INPUTS
% 
% figure
% %This figure says that all the graphs with the subplot
% %of U will be on the same page 
% 
% subplot(5,1,1)
% plot(t, u1)
% legend('u_1')
% grid on
% 
% subplot(5,1,2)
% plot(t, u2)
% legend('u_2')
% grid on
% 
% subplot(5,1,3)
% plot(t, u3)
% legend('u_3')
% grid on
% 
% subplot(5,1,4)
% plot(t, u4)
% legend('u_4')
% grid on
% 
% subplot(5,1,5)
% plot(t, u5)
% legend('u_5')
% grid on



%NOW PLOTTING STATES

% figure
% 
% subplot(9,1,1)
% plot(t, x1)
% legend('x_1')
% grid on
% 
% subplot(9,1,2)
% plot(t, x2)
% legend('x_2')
% grid on
% 
% subplot(9,1,3)
% plot(t, x3)
% legend('x_3')
% grid on
% 
% subplot(9,1,4)
% plot(t, x4)
% legend('x_4')
% grid on
% 
% subplot(9,1,5)
% plot(t, x5)
% legend('x_5')
% grid on
% 
% subplot(9,1,6)
% plot(t, x6)
% legend('x_6')
% grid on
% 
% subplot(9,1,7)
% plot(t, x7)
% legend('x_7')
% grid on
% 
% subplot(9,1,8)
% plot(t, x8)
% legend('x_8')
% grid on
% 
% subplot(9,1,9)
% plot(t, x9)
% legend('x_9')
% grid on
