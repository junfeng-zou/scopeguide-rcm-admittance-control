function closedCount = close_stage04_software_deadman_demo()
%CLOSE_STAGE04_SOFTWARE_DEADMAN_DEMO Force-close stale Stage 4 windows.
% This deliberately bypasses CloseRequestFcn, including callbacks whose
% function workspace was destroyed by Ctrl+C.

tag = 'ScopeGuideStage04SoftwareDeadmanDemo';
name = 'ScopeGuide Stage 4 Software Deadman (OFFLINE)';
byTag = findall(groot, 'Type', 'figure', 'Tag', tag);
byName = findall(groot, 'Type', 'figure', 'Name', name);
figures = unique([byTag(:); byName(:)]);
closedCount = numel(figures);
for index = 1:closedCount
    figureHandle = figures(index);
    if isgraphics(figureHandle)
        set(figureHandle, 'CloseRequestFcn', '', ...
            'WindowKeyPressFcn', '', 'WindowKeyReleaseFcn', '', ...
            'WindowButtonUpFcn', '');
        delete(figureHandle);
    end
end
if nargout == 0 && closedCount > 0
    fprintf('Closed %d Stage 4 software deadman window(s).\n', ...
        closedCount);
end
end
