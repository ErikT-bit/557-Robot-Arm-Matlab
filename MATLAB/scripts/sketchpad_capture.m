function strokes = sketchpad_capture()
% sketchpad_capture
% Fixed sketchpad for normalized drawing in [0,1]x[0,1]
%
% Left-click drag = draw
% Release = end stroke
% Press 'd' = done
%
% Fixes:
%   - no bogus first point at x=0
%   - captures actual click point on mouse-down
%   - ignores clicks outside axes
%   - avoids duplicate points if mouse hasn't moved

strokes = {};
cur = [];
drawing = false;

fig = figure( ...
    'Name','Sketchpad (drag to draw, press d when done)', ...
    'NumberTitle','off', ...
    'Color','w');

ax = axes(fig);
axis(ax, [0 1 0 1]);
axis(ax, 'equal');
axis(ax, 'manual');
grid(ax, 'on');
hold(ax, 'on');
title(ax, "Drag with LEFT mouse to draw. Release to end stroke. Press 'd' when done.");

set(fig, 'WindowButtonDownFcn',   @onDown);
set(fig, 'WindowButtonUpFcn',     @onUp);
set(fig, 'WindowButtonMotionFcn', @onMove);
set(fig, 'KeyPressFcn',           @onKey);

uiwait(fig);

    function onDown(~,~)
        if ~strcmp(get(fig,'SelectionType'),'normal')
            return;
        end

        pt = getClampedPoint();
        if isempty(pt)
            return;
        end

        drawing = true;
        cur = pt;   % start stroke with actual click point
        plot(ax, pt(1), pt(2), '.');
        drawnow limitrate
    end

    function onMove(~,~)
        if ~drawing
            return;
        end

        pt = getClampedPoint();
        if isempty(pt)
            return;
        end

        % avoid adding identical consecutive points
        if isempty(cur) || norm(pt - cur(end,:)) > 1e-6
            cur = [cur; pt]; %#ok<AGROW>
            plot(ax, pt(1), pt(2), '.');
            drawnow limitrate
        end
    end

    function onUp(~,~)
        if ~drawing
            return;
        end

        drawing = false;

        if size(cur,1) >= 2
            strokes{end+1} = cur; %#ok<AGROW>
            plot(ax, cur(:,1), cur(:,2), '-');
        elseif size(cur,1) == 1
            % keep single-click dots from being discarded if desired
            strokes{end+1} = cur; %#ok<AGROW>
            plot(ax, cur(:,1), cur(:,2), '.');
        end

        cur = [];
        drawnow
    end

    function onKey(~,evt)
        if strcmpi(evt.Key,'d')
            uiresume(fig);
            if isvalid(fig)
                close(fig);
            end
        end
    end

    function pt = getClampedPoint()
        % Get current point in axes coordinates.
        cp = get(ax,'CurrentPoint');
        x = cp(1,1);
        y = cp(1,2);

        % Ignore points far outside the sketch box
        if x < 0 || x > 1 || y < 0 || y > 1
            pt = [];
            return;
        end

        % Clamp for safety
        x = max(0, min(1, x));
        y = max(0, min(1, y));

        pt = [x y];
    end
end