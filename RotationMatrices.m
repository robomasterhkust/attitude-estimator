syms x y z
RX = [1 0 0;0 cos(x) -sin(x);0 sin(x) cos(x)];
RY = [cos(y) 0 sin(y);0 1 0;-sin(y) 0 cos(y)];
RZ = [cos(z) -sin(z) 0;sin(z) cos(z) 0;0 0 1];

RDX = [0 0 0;0 -sin(x) -cos(x);0 cos(x) -sin(x)];
RDY = [-sin(y) 0 cos(y);0 0 0;-cos(y) 0 -sin(y)];
RDZ = [-sin(z) -cos(z) 0;cos(z) -sin(z) 0;0 0 0];

RTX = [1 0 0;0 cos(x) sin(x);0 -sin(x) cos(x)];
RTY = [cos(y) 0 -sin(y);0 1 0;sin(y) 0 cos(y)];
RTZ = [cos(z) sin(z) 0;-sin(z) cos(z) 0; 0 0 1];
