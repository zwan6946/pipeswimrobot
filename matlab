
clc; 
clear; % see if clear workspace is needed or not
close all;
file_num = 20000; % file_num is equal to the lat file name is sd card
a=textread('100.txt');
x1=a(1:99 , 1:12);
for i = 100:100:file_num
    file_name = sprintf('%d%s',i,'.txt');
    a=textread(file_name);
    t=a(1:100 , 1:12);  
    x1=[x1',t']';
end
%% 

%sample_rate = 100; % SHOULD be EQUAL to the TRUE Data Logging Rate ! NOT Indicated data logging rate. 
beginfrom = 0;   %  (Unit: s) remove the unstable data from the beginning

radius = 0.05; % THE RADIUS OF THE BALL
gravity = 0.996;  % although acc is calibrated, here we calibrate it again according to resting position of center changes.

%ts = 1.00 / sample_rate;
ts = (x1(end,2)-x1(1,2))/(x1(end,1)/x1(1,1))/1000;

 b= fix(beginfrom/ts);
 if b==0  
     b=1;
 end
        
data = x1(b:end,:); 
%% 

[row,col]=size(data);
gx = 0;
gy = 0;
gz = 0;
angx_d = 0;
angy_d = 0;
angz_d = 0;     

% initialize zero arrays
ps_x = zeros(row+1,1);
ps_y = zeros(row+1,1);
ps_z = zeros(row+1,1);
distance = zeros(row+1,1);

v_x = zeros(row,1); % Calculated assuming pure rolling without translation.
v_y = zeros(row,1);
v_z = zeros(row,1);

ac_x = zeros(row,1); % global linear acceleration. 'c' for 'center'.
ac_y = zeros(row,1);
ac_z = zeros(row,1);
a = zeros(row,1);

vc_x = zeros(row+1,1); % Calculated based on global linear acceleration. 'c' for 'center'.
vc_y = zeros(row+1,1);
vc_z = zeros(row+1,1);

psc_x = zeros(row+2,1); % Calculated based on global linear acceleration. 'c' for 'center'.
psc_y = zeros(row+2,1);
psc_z = zeros(row+2,1);

wx = zeros(row,1);
wy = zeros(row,1);
wz = zeros(row,1);

yaw = zeros(row,1);
pitch = zeros(row,1);
roll = zeros(row,1);

time = data(:,2)/1000; %time is stored in columns 2
sound = data(:,3); %time is stored in columns 2

for i = 1:row % Main loop to CONVERT EULER ANGLE TO POSITION AS A PURE ROTATION

    roll(i) = data(i,12) * pi /180;  %radians
    pitch(i) = data(i,11) * pi /180; 
    yaw(i) = data(i,10) * pi /180;     %yaw = (data(i,6)-180) * pi /180; 
    microsNow = data(i,2);
    
    gx = data(i,7);
    gy = data(i,8);
    gz = data(i,9);% gyro readings in degree

    %calculates Rotaional Matrix Rotation 
    Rroll = [1 0 0 ; 0 cos(roll(i)) -sin(roll(i)); 0 sin(roll(i)) cos(roll(i))];
    Rpitch = [cos(pitch(i)) 0 sin(pitch(i)); 0 1 0; -sin(pitch(i)) 0 cos(pitch(i))];
    Ryaw = [cos(yaw(i)) -sin(yaw(i)) 0; sin(yaw(i)) cos(yaw(i)) 0; 0 0 1]; %i.e Identity matrix. !!Here we assume Yaw is constant at 180!!!
    % Pay attention: is the sign of yaw, pitch, roll correct?? Check with
    % simplest movement, such as, rotation by 90 degrees.
     
    Rotation = Ryaw * Rpitch * Rroll;
    Rotation_T = Rotation.';
    
    %Here we convert the gyro data from local to global, and obtain "global angular velocity 'w'" 
    wx(i) = Rotation(1,1) * gx + Rotation(1,2) * gy + Rotation(1,3) * gz ;  %degrees
    wy(i) = Rotation(2,1) * gx + Rotation(2,2) * gy + Rotation(2,3) * gz ;
    wz(i) = Rotation(3,1) * gx + Rotation(3,2) * gy + Rotation(3,3) * gz ;
    
    v_x(i) = wx(i) * radius * pi /180;  %radian
    v_y(i) = wy(i) * radius * pi /180; 
    % wz doesn't contribute to the moving of the ball.
    
    d_ps_x = v_x(i)*ts;
    d_ps_y = v_y(i)*ts;
    
    ps_x(i+1) = ps_x(i) + d_ps_x; % Though actually wx causes psy, and wy causes -psx. There's pi/2 phase delay.
    ps_y(i+1) = ps_y(i) + d_ps_y;
    
    distance(i+1) = distance(i) + sqrt(d_ps_x^2 + d_ps_y^2);
    
    %Below we calculate the position only based on global linear acceleration, which has nothing to do with rolling.
    ax = data(i,4);
    ay = data(i,5);
    az = data(i,6);% acc readings (local acceleration)
    a(i) = sqrt(ax^2 + ay^2 + az^2);
    
    ac_x(i) = Rotation(1,1) * ax + Rotation(1,2) * ay + Rotation(1,3) * az ;  % global acceleration.
    ac_y(i) = Rotation(2,1) * ax + Rotation(2,2) * ay + Rotation(2,3) * az ;
    ac_z(i) = Rotation(3,1) * ax + Rotation(3,2) * ay + Rotation(3,3) * az ;
    
    % tests implies that, yaw pitch roll (madgwick output) is already a
    % passive rotation. so simply: global = RM * local.
    
    vc_x(i+1) = vc_x(i) +  ac_x(i) *ts;
    vc_y(i+1) = vc_y(i) +  ac_y(i) *ts;
    vc_z(i+1) = vc_z(i) + (ac_z(i) - gravity) *ts;
    
    d_psc_x = vc_x(i+1)*ts;
    d_psc_y = vc_y(i+1)*ts;
    d_psc_z = vc_z(i+1)*ts;
    
    psc_x(i+2) = psc_x(i+1) + d_psc_x; % Twice integration. Delay for two steps: i ->> i+2.
    psc_y(i+2) = psc_y(i+1) + d_psc_y;
    psc_z(i+2) = psc_z(i+1) + d_psc_z;
    
end
v_x(1) = 0.01;
v_y(1) = 0.01; %for better comet graph display of v(vx, vy).
%%
figure(1)
hold on
grid on
xlabel('time (s)')
ylabel('distance (m)')
zlabel('high(m)')
title('pipe location')
plot3(time(1:end),distance(2:end),psc_z(3:end))
hold off

figure(2)
hold on
grid on
xlabel('distance (m)')
ylabel('high(m)')
title('pipe height')
plot(time(1:end),psc_z(3:end))
hold off
%%

 figure(1);
 subplot(2,3,1);
 hold on
 grid on
 xlabel('time (s)')
 ylabel('distance (m)')
 title('distance')
 plot (time(1:end),distance(2:end),'b')
 hold off

subplot(2,3,2);
hold on;
grid on
xlabel('time');
ylabel('dps');
title('Gyroscope readings');
plot(time(2:end),data(2:end,7), 'r');% data refers to .txt data
plot(time(2:end),data(2:end,8), 'g');
plot(time(2:end),data(2:end,9), 'b');
legend('gx', 'gy', 'gz');
hold off


subplot(2,3,3);
hold on;
grid on
xlabel('time');
ylabel('dps');
title('global Angular Velocity w');
plot(time(2:end),wx(2:end), 'r');
plot(time(2:end),wy(2:end), 'g');
plot(time(2:end),wz(2:end), 'b');
legend('wx', 'wy', 'wz');
hold off

subplot(2,3,4);
hold on;
grid on
title('Accelerometer readings');
xlabel('time');
ylabel('g');
plot(time(2:end),data(2:end,4), 'r');
plot(time(2:end),data(2:end,5), 'g');
plot(time(2:end),data(2:end,6), 'b');
plot(time(2:end),a(2:end), 'M');
legend('ax', 'ay', 'az','a'); % Let chip rest in several directions and see if 'a' changes. If not, then acc seems well calibrated.
hold off

subplot(2,3,5);
hold on;
grid on
xlabel('vx - m/s');
ylabel('vy - m/s');
title('Global Linear Velocity v by gyro');
comet(v_x,v_y);
hold off


subplot(2,3,6);
hold on;
grid on
plot(time(1:end),ac_x(1:end), 'r');
plot(time(1:end),ac_y(1:end), 'g');
plot(time(1:end),ac_z(1:end), 'b');
xlabel('time');
ylabel('acceleration/g');
title('global linear accleration by accelerometer');
legend('ac_x', 'ac_y', 'ac_z');
hold off
%% 

figure(2);
hold on;
grid on
xlabel('time');
ylabel('ps center');
title('position based on linear accleration');
plot(time(1:end),psc_x(3:end), 'r');
plot(time(1:end),psc_y(3:end), 'g');
plot(time(1:end),psc_z(3:end), 'b');
legend('psc_x', 'psc_y', 'psc_z');
hold off




figure(3); 
subplot(2,3,1);plot(time(2:end),180/pi*yaw(2:row));title('yaw');
subplot(2,3,2); plot(time(2:end),180/pi*pitch(2:row));title('pitch');
subplot(2,3,3); plot(time(2:end),180/pi*roll(2:row));title('roll');
subplot(2,3,4);plot(time(1:end),sound, 'r');title('sound');
subplot(2,3,5); comet(ps_x, ps_y); title('position');




