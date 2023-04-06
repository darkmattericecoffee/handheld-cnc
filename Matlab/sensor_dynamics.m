%% Sensor Rigid Body Dynamics
clear; clc;

syms v0x v0y v1x v1y v2x v2y v3x v3y l wb th real

V0 = [v0x v0y 0]';
V1 = [v1x v1y 0]';
V2 = [v2x v2y 0]';
V3 = [v3x v3y 0]';

X0 = [-l l 0]'/2;
X1 = [l l 0]'/2;
X2 = [-l -l 0]'/2;
X3 = [l -l 0]'/2;

w = [0 0 wb]';

% 
V(:,1) = V0 + cross(w, X1 - X0);
V(:,2) = V0 + cross(w, X2 - X0);
V(:,3) = V0 + cross(w, X3 - X0);
V(:,4) = V1 + cross(w, X2 - X1);
V(:,5) = V1 + cross(w, X3 - X1);
V(:,6) = V2 + cross(w, X3 - X2);

