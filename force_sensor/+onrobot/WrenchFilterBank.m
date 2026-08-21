classdef WrenchFilterBank < handle
    %WRENCHFILTERBANK Causal dual-branch filtering for HEX-H safety logic.
    %
    % Fast branch: first-order EMA, default cutoff 20 Hz.
    % Slow branch: second-order Butterworth, default cutoff 5 Hz.
    % No Signal Processing Toolbox is required.
    %
    % Example:
    %   filters = onrobot.WrenchFilterBank(200, 20, 5);
    %   [wFast, wSlow] = filters.step(sample.wrench);

    properties (SetAccess = private)
        SampleRateHz
        FastCutoffHz
        SlowCutoffHz
        IsInitialized = false
    end

    properties (Access = private)
        FastAlpha
        SlowB
        SlowA
        FastPrevious
        SlowX1
        SlowX2
        SlowY1
        SlowY2
    end

    methods
        function obj = WrenchFilterBank(sampleRateHz, fastCutoffHz, slowCutoffHz)
            if nargin < 1
                sampleRateHz = 200;
            end
            if nargin < 2
                fastCutoffHz = 20;
            end
            if nargin < 3
                slowCutoffHz = 5;
            end
            if fastCutoffHz <= 0 || fastCutoffHz >= sampleRateHz / 2 || ...
                    slowCutoffHz <= 0 || slowCutoffHz >= sampleRateHz / 2
                error('onrobot:WrenchFilterBank:InvalidCutoff', ...
                    'Cutoffs must lie between 0 and Nyquist frequency.');
            end

            obj.SampleRateHz = sampleRateHz;
            obj.FastCutoffHz = fastCutoffHz;
            obj.SlowCutoffHz = slowCutoffHz;
            obj.FastAlpha = 1 - exp(-2 * pi * fastCutoffHz / sampleRateHz);
            [obj.SlowB, obj.SlowA] = obj.butterworth2Lowpass( ...
                slowCutoffHz, sampleRateHz);
            obj.clearState();
        end

        function [wrenchFast, wrenchSlow] = step(obj, wrench)
            wrench = double(wrench(:));
            if numel(wrench) ~= 6 || any(~isfinite(wrench))
                error('onrobot:WrenchFilterBank:InvalidWrench', ...
                    'Input must be a finite 6-by-1 wrench vector.');
            end

            if ~obj.IsInitialized
                obj.reset(wrench);
                wrenchFast = wrench;
                wrenchSlow = wrench;
                return;
            end

            wrenchFast = obj.FastAlpha * wrench + ...
                (1 - obj.FastAlpha) * obj.FastPrevious;
            obj.FastPrevious = wrenchFast;

            wrenchSlow = obj.SlowB(1) * wrench + ...
                obj.SlowB(2) * obj.SlowX1 + ...
                obj.SlowB(3) * obj.SlowX2 - ...
                obj.SlowA(2) * obj.SlowY1 - ...
                obj.SlowA(3) * obj.SlowY2;

            obj.SlowX2 = obj.SlowX1;
            obj.SlowX1 = wrench;
            obj.SlowY2 = obj.SlowY1;
            obj.SlowY1 = wrenchSlow;
        end

        function reset(obj, initialWrench)
            if nargin < 2
                obj.clearState();
                return;
            end
            initialWrench = double(initialWrench(:));
            if numel(initialWrench) ~= 6 || any(~isfinite(initialWrench))
                error('onrobot:WrenchFilterBank:InvalidInitialWrench', ...
                    'Initial wrench must be a finite 6-by-1 vector.');
            end
            obj.FastPrevious = initialWrench;
            obj.SlowX1 = initialWrench;
            obj.SlowX2 = initialWrench;
            obj.SlowY1 = initialWrench;
            obj.SlowY2 = initialWrench;
            obj.IsInitialized = true;
        end

        function delay = estimatedDelayMs(obj)
            delay = struct();
            delay.fast = 1000 * (1 - obj.FastAlpha) / ...
                (obj.FastAlpha * obj.SampleRateHz);
            slowDelaySamples = sqrt(2) / ...
                (2 * tan(pi * obj.SlowCutoffHz / obj.SampleRateHz));
            delay.slow = 1000 * slowDelaySamples / obj.SampleRateHz;
        end
    end

    methods (Access = private)
        function clearState(obj)
            obj.FastPrevious = zeros(6, 1);
            obj.SlowX1 = zeros(6, 1);
            obj.SlowX2 = zeros(6, 1);
            obj.SlowY1 = zeros(6, 1);
            obj.SlowY2 = zeros(6, 1);
            obj.IsInitialized = false;
        end
    end

    methods (Static, Access = private)
        function [b, a] = butterworth2Lowpass(cutoffHz, sampleRateHz)
            k = tan(pi * cutoffHz / sampleRateHz);
            normalizer = 1 / (1 + sqrt(2) * k + k^2);
            b0 = k^2 * normalizer;
            b = [b0, 2 * b0, b0];
            a = [1, 2 * (k^2 - 1) * normalizer, ...
                (1 - sqrt(2) * k + k^2) * normalizer];
        end
    end
end
