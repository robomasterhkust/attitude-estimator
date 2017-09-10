clear all
%% parameters
w_gyro = 0.15;
w_accel = 0.3;
bias_max = 0.05;

r_gyro = single(0.0008);
r_accel = single(0.36);
r_mag = single(100);
dt = single(0.005);
add_noise = true;
add_gyro_bias = true;

%Test Parameters
q_real = [1 0 0 0]';
R = eye(3);
eulerAngles_real = [0 0 0];
z = [0 0 0 0 0 9.81 0.2 -0.2 0.2]';
zr = z;
zf = zeros(9,3);
gyro_bias = [0.04 0.035 -0.02]';
......params [Gyro(Roll Pitch Yaw) Accel(x y z) Mag(null<3>)]......
len = 5000;
w_E = [[0.1     0.2     0.5]; 
       [  1     0.2   -0.03]; 
       [ -4    0.01     0.2]; 
       [ -3      -5       2]; 
       [  0   -0.02    0.05]];

%Filter option
cutoff_freq = 30;
sample_freq = 1/dt;

fr = sample_freq / cutoff_freq;
ohm = tan(pi / fr);
c = 1.0 + 2.0 * cos(pi / 4.0) * ohm + ohm * ohm;
b0 = ohm * ohm / c;
b1 = 2.0 * b0;
b2 = b0;
a1 = 2.0 * (ohm * ohm - 1.0) / c;
a2 = (1.0 - 2.0 * cos(pi / 4.0) * ohm + ohm * ohm) / c;


%% initialize test
setpoint = zeros(3,len);
result = zeros(3,len);
theta_gyro = sqrt(r_gyro);
theta_accel = sqrt(r_accel);
theta_mag = sqrt(r_mag);
count = 0;

w_len = len/length(w_E(:,1));
%% test
for i = 1:len
zprev = zr;    
if add_noise == true
    zr = zeros(9,1);
    for j = 1:3
        zr(j) = z(j) + normrnd(0,theta_gyro);
        zr(j+3) = z(j+3) + normrnd(0,theta_accel);
        zr(j+6) = z(j+6) + normrnd(0,theta_mag);
    end
else
    zr = z;
end

if add_gyro_bias == true
    zr(1:3) = zr(1:3) + gyro_bias;
end

%Apply filter
zf(:,1) = zr - zf(:,2) * a1 - zf(:,3) * a2;
zr = zf(:,1) * b0 + zf(:,2) * b1 + zf(:,3) * b2;

zf(:,3) = zf(:,2);
zf(:,2) = zf(:,1);

%Attitude estimator
[eulerAngles,R] = AttitudeQ(dt,zr,w_gyro,w_accel,bias_max);

%regenerate test scenario
%dq = [0.1 -0.04 -0.02 0.001]';
   
w = R\(w_E(uint8((count+w_len/2)/w_len),:)');
dq = 0.5*[q_real(1) -q_real(2) -q_real(3) -q_real(4);
		  q_real(2)  q_real(1) -q_real(4)  q_real(3);
		  q_real(3)  q_real(4)  q_real(1) -q_real(2);
		  q_real(4) -q_real(3)  q_real(2)  q_real(1)]*[0;w];

q_real = q_real + dq*dt;
q_real = q_real/norm(q_real);
z = imuFakeInputQ(q_real,dq);
         
%phi=atan2(2 * (q_real(1) * q_real(2) + q_real(3) * q_real(4)), 1 - 2 * (q_real(2) * q_real(2) + q_real(3) * q_real(3)));
%theta=asin(2 * (q_real(1) * q_real(3) - q_real(4) * q_real(2)));
%psi=atan2(2 * (q_real(1) * q_real(4) + q_real(2) * q_real(3)), 1 - 2 * (q_real(3) * q_real(3) + q_real(4) * q_real(4)));

R = [q_real(1)*q_real(1) + q_real(2)*q_real(2) - q_real(3)*q_real(3) - q_real(4)*q_real(4)...
     2*(q_real(2) * q_real(3) - q_real(1) * q_real(4))...
     2*(q_real(1) * q_real(3) + q_real(2) * q_real(4));
     2*(q_real(2) * q_real(3) + q_real(1) * q_real(4))...
     q_real(1)*q_real(1) - q_real(2)*q_real(2) + q_real(3)*q_real(3) - q_real(4)*q_real(4)...
     2*(q_real(3) * q_real(4) - q_real(1) * q_real(2));
     2*(q_real(2) * q_real(4) - q_real(1) * q_real(3))...
     2*(q_real(1) * q_real(2) + q_real(3) * q_real(4))...
     q_real(1)*q_real(1) - q_real(2)*q_real(2) - q_real(3)*q_real(3) + q_real(4)*q_real(4)];

phi=asin(R(3,2));
theta=-atan2(R(3,1),R(3,3));
psi=-atan2(R(1,2),R(2,2));
eulerAngles_real=[phi;theta;psi];

setpoint(:,i) = eulerAngles_real * 180/pi;
result(:,i) = eulerAngles * 180/pi;
count = count + 1;
end

%% generate plot
xaxis = (1:len)*dt;
figure();
subplot(3,1,1)
hold on
plot(xaxis,result(1,:),'r','LineWidth',3);
plot(xaxis,setpoint(1,:),'b');
hold off
subplot(3,1,2)
hold on
plot(xaxis,result(2,:),'r','LineWidth',3);
plot(xaxis,setpoint(2,:),'b');
hold off
subplot(3,1,3)
hold on
plot(xaxis,result(3,:),'r','LineWidth',3);
plot(xaxis,setpoint(3,:),'b');
hold off
