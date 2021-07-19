function x = tuker(x,y,z)
%TUKER Summary of this function goes here
%   Detailed explanation goes here
    x(:,4)=x(:,y);
    x(:,y)=x(:,z);
    x(:,z)=x(:,4);
    x=x(:,1:3);
end

