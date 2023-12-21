function [r, speed, center, initialYaw, numRevs] = trajectory(r, speed, center, initialYaw, numRevs)

% % Trajectory parameters
% r = 8.42; % (m)
% speed = 2.50; % (m/s)
% center = [0, 0]; % (m)
% initialYaw = 90; % (degrees)
% numRevs = 2;

    % Define angles theta and corresponding times of arrival t.
    revTime = 2*pi*r / speed;
    theta = (0:pi/2:2*pi*numRevs).';
    t = linspace(0, revTime*numRevs, numel(theta)).';
    
    % Define position.
    x = r .* cos(theta) + center(1);
    y = r .* sin(theta) + center(2);
    z = zeros(size(x));
    position = [x, y, z];
    
    % Define orientation.
    yaw = theta + deg2rad(initialYaw);
    yaw = mod(yaw, 2*pi);
    pitch = zeros(size(yaw));
    roll = zeros(size(yaw));
    orientation = quaternion([yaw, pitch, roll], 'euler', ...
        'ZYX', 'frame');
    

   