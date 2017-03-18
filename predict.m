function [x,P]= predict (x,P,v,g,Q,WB,dt)
%function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   xn, Pn - predicted state and covariance
%
% Tim Bailey 2004.



% jacobians   
Gv= [   
        1 0 -v*dt*sin(x(3)+g);
        0 1 v*dt*cos(x(3)+g);
        0 0 1
    ]
Gu= [
        dt*cos(x(3)+g)  -v*dt*sin(x(3)+g);
        dt*sin(x(3)+g)  v*dt*cos(x(3)+g);
        dt*sin(g)/WB     v*dt*cos(g)/WB
    ]
  
% predict covariance
P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
if size(P,1)>3
    P(1:3,4:end)= Gv*P(1:3,4:end);
    P(4:end,1:3)= P(1:3,4:end)';
end    

% predict state
x(1:3)= [   x(1) + v*dt*cos(x(3)+g)
            x(2) + v*dt*sin(x(3)+g)
            x(3) + v*dt*sin(g)/WB
        ]

