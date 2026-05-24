% using the OL found from the research paper, (5/7 * g)/s^2
m = .06735;
r = .0127;


s = tf('s');
sys_OL = 7.00714/s^2;

% setup variables and loop
t = 0:.01:10;
K = 1.5:.025:2.5;      % lower Kp
A = 0.01:.01:0.25;     % lower Ki
B = 0.1:.025:1.5;      % higher Kd

totalRuns = length(K)*length(A)*length(B);
runCount = 0;
tic

i = 1; % index variable
results = []; % store gains and performance chars here
for Kp = K
    for Ki = A
        for Kd = B

            runCount = runCount + 1;

            if mod(runCount,10000) == 0
                fprintf("Progress: %.2f%% | Run %d of %d | Time: %.1f s\n", ...
                    100*runCount/totalRuns, runCount, totalRuns, toc);
            end

            %C = k*(s+a)*(s+b); % Compensator C, a PID controller
            C = (Kp*s + Ki + Kd*s^2)/s;
            sys_CL = feedback(C*sys_OL,1); % closed loop tf with compensator
            if isstable(sys_CL)
                y = step(sys_CL,t);
                perf_chars = stepinfo(y,t,1);
                tr = perf_chars.RiseTime;
                ts = perf_chars.SettlingTime;
                os = perf_chars.Overshoot;
                if (tr <= 1 && ts <= 3 && os <= 10)
                    % Store performance characteristics for each combination of gains
                    results(i, :) = [Kp, Ki, Kd, tr, ts, os];
                    i = i + 1; % Increment index for results storage    
                end
            end
        end
    end
end

if isempty(results)
    disp("No parameter combinations met the requirements.")
else
    fprintf('\nAll valid parameter sets:\n\n');
    fprintf('   Kp      Ki      Kd      RiseTime   SettlingTime   Overshoot\n');

    for n = 1:size(results,1)
        fprintf('%5.2f  %5.2f  %5.2f   %8.3f    %8.3f      %8.3f\n', ...
            results(n,1), results(n,2), results(n,3), ...
            results(n,4), results(n,5), results(n,6));
    end

    writematrix(results,'kpkikd_withardpara_as_start.csv');

    [~, idx] = min(results(:,6));
    optimalParams = results(idx, :);

    disp("Best Parameters: ")
    disp(optimalParams(1:3));
    disp("Rise Time: " + optimalParams(4));
    disp("Settling Time: " + optimalParams(5));
    disp("Overshoot: " + optimalParams(6));

    fprintf("Rise time range: %.4f to %.4f\n", min(results(:,4)), max(results(:,4)));
    fprintf("Settling time range: %.4f to %.4f\n", min(results(:,5)), max(results(:,5)));
    fprintf("Overshoot range: %.4f to %.4f\n", min(results(:,6)), max(results(:,6)));
end

