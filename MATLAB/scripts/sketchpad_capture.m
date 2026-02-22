function strokes = sketchpad_capture()
% sketchpad_capture
% Fixed axis limits (0..1) to avoid MATLAB auto-scaling weirdness.
% Left-click drag = draw. Release = end stroke.
% Press 'd' to finish.

strokes = {};
cur = [];
drawing = false;

fig = figure('Name','Sketchpad (drag to draw, press d when done)','NumberTitle','off');
ax = axes(fig);
axis(ax, [0 1 0 1]);
axis(ax, 'equal');
grid(ax, 'on');
hold(ax, 'on');
title(ax, "Drag with LEFT mouse to draw. Release to end stroke. Press 'd' when done.");

set(fig, 'WindowButtonDownFcn', @onDown);
set(fig, 'WindowButtonUpFcn',   @onUp);
set(fig, 'WindowButtonMotionFcn', @onMove);
set(fig, 'KeyPressFcn', @onKey);

uiwait(fig);

    function onDown(~,~)
        if ~strcmp(get(fig,'SelectionType'),'normal')
            return;
        end
        drawing = true;
        cur = [];
        onMove();
    end

    function onMove(~,~)
        if ~drawing, return; end
        pt = get(ax,'CurrentPoint');
        x = pt(1,1); y = pt(1,2);
        x = max(0, min(1, x));
        y = max(0, min(1, y));
        cur = [cur; x y]; %#ok<AGROW>
        plot(ax, x, y, '.');
        drawnow limitrate
    end

    function onUp(~,~)
        if ~drawing, return; end
        drawing = false;
        if size(cur,1) >= 2
            strokes{end+1} = cur; %#ok<AGROW>
            plot(ax, cur(:,1), cur(:,2), '-');
        end
        cur = [];
        drawnow
    end

    function onKey(~,evt)
        if strcmpi(evt.Key,'d')
            uiresume(fig);
            if isvalid(fig), close(fig); end
        end
    end
end
