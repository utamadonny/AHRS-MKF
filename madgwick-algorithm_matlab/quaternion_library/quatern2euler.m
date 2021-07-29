function euler = quatern2euler(q)
%QUATERN2EULER Converts a quaternion orientation to ZYX Euler angles
%
%   q = quatern2euler(q)
%
%   Converts a quaternion orientation to ZYX Euler angles where phi is a
%   rotation around X, theta around Y and psi around Z.
%
%   For more information see:
%   https://x-io.co.uk/quaternions/
%
%	Date          Author          Notes
%	27/09/2011    SOH Madgwick    Initial release
%   21/07/2021    Donny           Modify quaternion to euler
% 
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
    psi = atan2(R(2,1,:), R(1,1,:) );

%   https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
%   https://automaticaddison.com/wp-content/uploads/2020/09/quaternion-to-rotation-matrix.jpg
%     D(1,1,:) = -2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
%     D(2,1,:) = q(:,1).*q(:,2)+q(:,3).*q(:,4);
%     D(2,2,:) = 0.5-q(:,2).*q(:,2)-q(:,3).*q(:,3);
%     D(3,1,:) = q(:,2).*q(:,3)+q(:,1).*q(:,4);
%     D(3,2,:) = 0.5-q(:,3).*q(:,3)-q(:,4).*q(:,4);
%     
%     phi = atan2(D(2,1,:),D(2,2,:));
%     theta = asin(D(1,1,:));
%     psi = atan2(D(3,1,:),D(3,2,:));

    euler = [phi(1,:)' theta(1,:)' psi(1,:)'];
end

