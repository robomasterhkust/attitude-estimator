function [eulerAngles,R] = AttitudeQ(dt,z,w_gyro,w_accel,bias_max)

%% init
persistent q
if(isempty(q))
    q = [1 0 0 0]';
end

persistent gyro_bias
if(isempty(gyro_bias))
    gyro_bias = zeros(3,1);
end
%% update

k = [2 * (q(2) * q(4) - q(1) * q(3));
     2 * (q(3) * q(4) + q(1) * q(2));
     q(1) * q(1) - q(2) * q(2) - q(3) * q(3) + q(4) * q(4)];
corr = cross(z(4:6)/norm(z(4:6)),k) * w_accel;
if(norm(z(1:3)) < 0.175)
    gyro_bias = gyro_bias + corr * w_gyro * dt;
    
    for i = 1:3
        if gyro_bias(i) > bias_max
            gyro_bias(i) = bias_max;
        end
        if gyro_bias(i) < -bias_max
            gyro_bias(i) = -bias_max;
        end
    end
end

rate = z(1:3)+gyro_bias;
corr = corr + rate;

q = q + 0.5*[q(1) -q(2) -q(3) -q(4);
			 q(2)  q(1) -q(4)  q(3);
			 q(3)  q(4)  q(1) -q(2);
			 q(4) -q(3)  q(2)  q(1)]*[0;corr]*dt;
q = q/norm(q);
         
R = [q(1) * q(1) + q(2) * q(2) - q(3) * q(3) - q(4) * q(4)...
     2.0 * (q(2) * q(3) - q(1) * q(4))...
     2.0 * (q(1) * q(3) + q(2) * q(4));
     2.0 * (q(2) * q(3) + q(1) * q(4))...
     q(1) * q(1) - q(2) * q(2) + q(3) * q(3) - q(4) * q(4)...
     2.0 * (q(3) * q(4) - q(1) * q(2));
     2.0 * (q(2) * q(4) - q(1) * q(3))...
     2.0 * (q(1) * q(2) + q(3) * q(4))...
     q(1) * q(1) - q(2) * q(2) - q(3) * q(3) + q(4) * q(4)];

phi=asin(R(3,2));
theta=-atan2(R(3,1),R(3,3));
psi=-atan2(R(1,2),R(2,2));
eulerAngles=[phi;theta;psi];