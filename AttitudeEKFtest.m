clear all

q_rotSpeed = single(1e-4);
q_rotAcc = single(0.08);
q_acc = single(0.009);
q_mag = single(0.005);
r_gyro = single(0.0008);
r_accel = single(10);
r_mag = single(100);
J = [0.0018 0 0;0 0.0018 0;0 0 0.0037];
dt = single(0.01);
approx_prediction = false;
use_inertia_matrix = false;
add_noise = true;
zFlag = [true true false];

%Test Parameters
q_real = [1 0 0 0]';
R = eye(3);
eulerAngles_real = [0 0 0];
z = [0 0 0 0 0 9.81 0.2 -0.2 0.2]';
zr = z;
......params [Gyro(Roll Pitch Yaw) Accel(x y z) Mag(null<3>)]......

%Initialize test
length = 5000;
setpoint = zeros(3,length);
result = zeros(3,length);
theta_gyro = sqrt(r_gyro);
theta_accel = sqrt(r_accel);
theta_mag = sqrt(r_mag);
for i = 1:length

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

    [xa_apo,Pa_apo,Rot_matrix,eulerAngles,debugOutput]...
 = AttitudeEKF(approx_prediction,use_inertia_matrix,zFlag,dt,zr,...
 q_rotSpeed,q_rotAcc,q_acc,q_mag,r_gyro,r_accel,r_mag,J);

%regenerate test scenario
w_E = [1 0.5 2]';
w = R\w_E;
dq = 0.5*[q_real(1) -q_real(2) -q_real(3) -q_real(4);
		  q_real(2)  q_real(1) -q_real(4)  q_real(3);
		  q_real(3)  q_real(4)  q_real(1) -q_real(2);
		  q_real(4) -q_real(3)  q_real(2)  q_real(1)]*[0;w];

q_real = q_real + dq*dt;
q_real = q_real/norm(q_real);
z = imuFakeInputQ(q_real,dq);
  
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
end

xaxis = (1:length)*dt;
figure();
subplot(3,1,1)
hold on
plot(xaxis,result(1,:),'r','LineWidth',3);
plot(xaxis,-setpoint(1,:),'b');
hold off
subplot(3,1,2)
hold on
plot(xaxis,result(2,:),'r','LineWidth',3);
plot(xaxis,-setpoint(2,:),'b');
hold off
subplot(3,1,3)
hold on
plot(xaxis,result(3,:),'r','LineWidth',3);
plot(xaxis,-setpoint(3,:),'b');
hold off
