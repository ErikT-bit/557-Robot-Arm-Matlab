function poses = strokes_to_taskspace(strokes, plane, penUpHeight)
% Convert 2D strokes into a list of SE(3) poses on the writing plane.
% Uses a fixed tool orientation relative to the plane.
%
% Tool z-axis points INTO the board (negative plane normal).
% After each stroke we insert a pen-up move along +plane.z_hat.

poses = {};

% tool orientation from plane frame
z_tool = -plane.z_hat;              % into board
x_tool = plane.x_hat;
y_tool = Normalize(cross(z_tool, x_tool));
x_tool = Normalize(cross(y_tool, z_tool));

R = [x_tool, y_tool, z_tool];

for k = 1:numel(strokes)
    xy = strokes{k}.xy; % 2xN (meters)

    % Pen-down poses along the stroke
    for i = 1:size(xy,2)
        p = plane.origin_pressed + xy(1,i)*plane.x_hat + xy(2,i)*plane.y_hat;
        T = [R, p; 0 0 0 1];
        poses{end+1} = T; %#ok<AGROW>
    end

    % Pen-up after each stroke
    p_last = poses{end}(1:3,4);
    p_up   = p_last + penUpHeight * plane.z_hat;
    poses{end+1} = [R, p_up; 0 0 0 1]; %#ok<AGROW>
end
end