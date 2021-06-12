function [R] = convertQuatToRotationMatrix(quat)
% CONVERTQUATTOROTATIONMATRIX Converts From Quaternion To Equivelent
% Rotation Matrix - Rotates a vector defined by the inertial frame to the
% body frame (rotates to match the angles defined by the quaternions)
% Assumes unit Quaternion As An Input

R = [(quat(1)^2 + quat(2)^2 - quat(3)^2 - quat(4)^2), 2*(quat(2)*quat(3)+quat(1)*quat(4)), 2*(quat(2)*quat(4)-quat(1)*quat(3));
      2*(quat(2)*quat(3)-quat(1)*quat(4)), (quat(1)^2 - quat(2)^2 + quat(3)^2 - quat(4)^2), 2*(quat(3)*quat(4)+quat(1)*quat(2));
      2*(quat(2)*quat(4)+quat(1)*quat(3)), 2*(quat(3)*quat(4)-quat(1)*quat(2)), (quat(1)^2 - quat(2)^2 - quat(3)^2 + quat(4)^2)];

end