function q = quest(vb, vi, weights)    %QUEST implementation of the QUEST algorithm
% Inputs
% ======
% vb: 3rowx3col matrix, with vectors as columns
%     Measurements in the spacecraft reference frame of the
%     spacecraft
%
% vi: 3rowx3col matrix, with vectors as columns
%     Measurements in the external inertial frame
%
% weights: 1rowx3col vector
%     weights associated to the three vectors 
%
% Returns
% =======
%
% q: 1rowx4col matrix
%    the optimised quaternion to convert from the spacecraft frame to the
%    inertial frame

B = zeros(3, 3);
for i = 1:3
   B = B + weights(i) * vb(:, i) * transpose(vi(:, i));  
end

S = B + transpose(B);
Z = [B(2, 3) - B(3, 2); B(3, 1) - B(1, 3); B(1, 2) - B(2, 1)];
sigma = trace(B);
optimal_eigenval = sum(weights);

p_invert = (optimal_eigenval + sigma) * eye(3, 3) - S;
p = inv(p_invert) * Z;
quat = (1.0 / sqrt(1 + dot(p, p))) * [p; 1];
q = quat2rotm(transpose(quat));  