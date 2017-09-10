function z_t = imuFakeInput(euler_angle,d_euler_angle)
   z_t = zeros(9,1);
   
   phi = euler_angle(1);
   theta = euler_angle(2);
   psi = euler_angle(3);
   RX = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
   RY = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
   RZ = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
   R = RZ*RX*RY;
   
   z_t(1:3) = [cos(theta)   0   -cos(phi)*sin(theta);
                        0   1               sin(phi);
               sin(theta)   0    cos(phi)*cos(theta)]*d_euler_angle;
   z_t(4:6) = R\[0 0 9.81]'; 
   z_t(7:9) = normrnd(0,100,[3 1]);
end

