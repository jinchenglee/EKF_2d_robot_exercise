function [z,H]= observe_model(x, idf)
%function [z,H]= observe_model(x, idf)
%
% INPUTS:
%   x - state vector
%   idf - index of feature order in state
%
% OUTPUTS:
%   z - predicted observation
%   H - observation Jacobian
%
% Given a feature index (ie, the order of the feature in the state vector),
% predict the expected range-bearing observation of this feature and its Jacobian.
%
% Tim Bailey 2004.

Nxv= 3; % number of vehicle pose states
fpos= Nxv + idf*2 - 1; % position of xf in state
H= zeros(2, length(x));

% distance from robot to current beacon
D_x = x(fpos)-x(1)
D_y = x(fpos+1)-x(2)
D = sqrt(D_x^2+D_y^2);

% predict z
z=  [
        D;
        atan(D_y/D_x)-x(3)
    ]

% calculate H
H(:,1:3)        = [
        -D_x/D  -D_y/D      0;
        D_y/D^2 -D_x/D^2    -1
    ]
H(:,fpos:fpos+1)= [
        D_x/D       D_y/D
        -D_y/D^2    D_x/D^2
    ]
