function [x,P]= update(x,P,z,R,idf)
% function [x,P]= update(x,P,z,R,idf)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%
% Outputs:
%   x, P - updated state and covariance


function [x,P]= single_update(x,P,z,R,idf)

lenz= size(z,2);
for i=1:lenz
    [zp,H]= observe_model(x, idf(i));
    
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    [x,P]= KF_simple_update(x,P,v,RR,H);
end        
