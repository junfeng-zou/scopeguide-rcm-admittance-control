function guidance = stage07FaultGuidance(timeSec, profile)
%STAGE07FAULTGUIDANCE Operator-facing countdown and recovery instructions.

arguments
    timeSec (1, 1) double {mustBeFinite, mustBeNonnegative}
    profile (1, 1) string {mustBeMember(profile, ...
        ["none", "acceptance"])} = "none"
end

guidance = emptyGuidance(profile);
if profile ~= "acceptance"
    return;
end

schedule = scopeguide.runtime.stage07FaultSchedule();
count = numel(schedule.TimeSec);
guidance.Enabled = true;

active = find(timeSec >= schedule.TimeSec & ...
    timeSec < schedule.TimeSec + schedule.DurationSec, 1, 'first');
if ~isempty(active)
    guidance.Phase = "ACTIVE";
    guidance.EventIndex = active;
    guidance.EventTimeSec = schedule.TimeSec(active);
    guidance.EventCode = schedule.StatusCode(active);
    guidance.EventLabel = schedule.ShortLabel(active);
    guidance.CountdownSec = 0;
    guidance.MessageZh = string(sprintf( ...
        '正在注入 %d/%d：%s（%s）｜应立即进入 FAULT，预测速度归零。', ...
        active, count, char(schedule.NameZh(active)), ...
        char(schedule.ShortLabel(active))));
    guidance.TitleText = string(sprintf('%s ACTIVE', ...
        char(schedule.ShortLabel(active))));
    guidance.ConsoleToken = "ACTIVE_" + string(active);
    return;
end

next = find(timeSec < schedule.TimeSec, 1, 'first');
if isempty(next)
    guidance.Phase = "RECOVER";
    guidance.EventIndex = count;
    guidance.MessageZh = ...
        "第五项注入已结束：撤力 -> Esc -> 等待至少 0.5 s -> R -> 确认蓝色。";
    guidance.TitleText = "FINAL FAULT RECOVERY";
    guidance.ConsoleToken = "RECOVER_5";
    return;
end

countdown = schedule.TimeSec(next) - timeSec;
guidance.EventIndex = next;
guidance.EventTimeSec = schedule.TimeSec(next);
guidance.EventCode = schedule.StatusCode(next);
guidance.EventLabel = schedule.ShortLabel(next);
guidance.CountdownSec = countdown;
guidance.TitleText = string(sprintf('next=%s T-%.1f s', ...
    char(schedule.ShortLabel(next)), countdown));

if countdown <= 5
    guidance.Phase = "PREPARE";
    guidance.MessageZh = string(sprintf( ...
        '准备 %d/%d：%s，T-%.1f s｜现在按一次 Space 置 ON，进入绿色后持续施加约 3–5 N 外力。', ...
        next, count, char(schedule.NameZh(next)), countdown));
    guidance.ConsoleToken = "PREPARE_" + string(next);
elseif next > 1
    guidance.Phase = "RECOVER";
    guidance.MessageZh = string(sprintf( ...
        '恢复：撤力 -> Esc -> 等待至少 0.5 s -> R -> 确认蓝色。下一项 %s @ %.0f s，剩余 %.1f s。', ...
        char(schedule.NameZh(next)), schedule.TimeSec(next), countdown));
    guidance.ConsoleToken = "RECOVER_" + string(next - 1);
else
    guidance.Phase = "WAIT";
    guidance.MessageZh = string(sprintf( ...
        '下一项 1/%d：%s @ %.0f s，剩余 %.1f s；T-5 s 时会提示使能和施力。', ...
        count, char(schedule.NameZh(next)), schedule.TimeSec(next), ...
        countdown));
end
end

function guidance = emptyGuidance(profile)
guidance = struct();
guidance.Enabled = false;
guidance.Profile = profile;
guidance.Phase = "DISABLED";
guidance.EventIndex = 0;
guidance.EventTimeSec = NaN;
guidance.EventCode = "";
guidance.EventLabel = "";
guidance.CountdownSec = NaN;
guidance.MessageZh = "";
guidance.TitleText = "";
guidance.ConsoleToken = "";
end
