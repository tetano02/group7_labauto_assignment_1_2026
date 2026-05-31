%% Confronto effetto filtraggio per feedforward
% Lo script confronta tre prove della stessa traiettoria:
%   1) senza filtro
%   2) filtro leggero
%   3) filtro piu' marcato
%
% Vengono mostrati solo:
%   - segnali temporali: velocita', accelerazione stimata, forza/coppia
%   - indice RMS del residuo rapido, usato come parametro di rumore

clc; clear; close all;

model_name = 'gantry_portal_sea_soft';
script_folder = fileparts(mfilename('fullpath'));
tests_folder = fullfile(script_folder, '..', model_name, 'tests');

%% Parametri da modificare
test_files = {
    'trajectory_20260523103058.mat'
    'trajectory_20260523103158.mat'
    'trajectory_20260523103451.mat'
};

case_labels = {
    'Senza filtro'
    'Filtro LP \tau = 0.001 s'
    'Filtro LP \tau = 0.005 s'
};

% Ordine: dal meno filtrato al piu' filtrato.
joints_to_plot = 1:3;

% Scegliere una porzione in cui l'effetto filtrante si vede bene.
% Usare [] per plottare tutta la traiettoria.
time_window = [8 10];

% Finestra usata per separare trend lento e residuo rapido.
% Il residuo rapido e' il segnale usato per stimare il "rumore".
trend_window_s = 0.05;

%% Caricamento dati
n_cases = numel(test_files);
time = cell(n_cases, 1);
velocity = cell(n_cases, 1);
acceleration = cell(n_cases, 1);
force = cell(n_cases, 1);
fs = zeros(n_cases, 1);

for icase = 1:n_cases
    file_path = fullfile(tests_folder, test_files{icase});
    data = load(file_path);

    time{icase} = data.time(:);
    fs(icase) = 1 / median(diff(time{icase}));

    velocity{icase} = data.joint_velocity;
    force{icase} = data.joint_torque;

    if size(velocity{icase}, 1) ~= numel(time{icase})
        velocity{icase} = velocity{icase}.';
    end
    if size(force{icase}, 1) ~= numel(time{icase})
        force{icase} = force{icase}.';
    end

    if isfield(data, 'joint_acceleration')
        acceleration{icase} = data.joint_acceleration;
        if size(acceleration{icase}, 1) ~= numel(time{icase})
            acceleration{icase} = acceleration{icase}.';
        end
    else
        dt = median(diff(time{icase}));
        acceleration{icase} = zeros(size(velocity{icase}));
        acceleration{icase}(1, :) = (velocity{icase}(2, :) - velocity{icase}(1, :)) / dt;
        acceleration{icase}(end, :) = (velocity{icase}(end, :) - velocity{icase}(end-1, :)) / dt;
        acceleration{icase}(2:end-1, :) = ...
            (velocity{icase}(3:end, :) - velocity{icase}(1:end-2, :)) / (2 * dt);
    end
end

signals = {velocity, acceleration, force};
signal_names = {'Velocita''', 'Accelerazione stimata', 'Forza/coppia'};
ylabels = {'dq', 'ddq', 'F'};

%% Grafici temporali
for ijoint = joints_to_plot
    figure('Name', sprintf('Confronto segnali J%d', ijoint), 'Color', 'w');
    tiledlayout(numel(signals), n_cases, ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    for isignal = 1:numel(signals)
        % Stessi limiti verticali per i tre casi, cosi' il confronto e' corretto.
        y_all = [];
        for icase = 1:n_cases
            idx = true(size(time{icase}));
            if ~isempty(time_window)
                idx = time{icase} >= time_window(1) & time{icase} <= time_window(2);
            end
            y_all = [y_all; signals{isignal}{icase}(idx, ijoint)]; %#ok<AGROW>
        end
        y_min = min(y_all);
        y_max = max(y_all);
        y_margin = 0.05 * max(y_max - y_min, eps);

        for icase = 1:n_cases
            nexttile((isignal - 1) * n_cases + icase);
            hold on; grid on;

            idx = true(size(time{icase}));
            if ~isempty(time_window)
                idx = time{icase} >= time_window(1) & time{icase} <= time_window(2);
            end

            plot(time{icase}(idx), signals{isignal}{icase}(idx, ijoint), ...
                'LineWidth', 1.0, ...
                'Color', [0.0000 0.4470 0.7410]);

            ylim([y_min - y_margin, y_max + y_margin]);

            if isignal == 1
                title(case_labels{icase});
            end
            if icase == 1
                ylabel(sprintf('%s\n%s', signal_names{isignal}, ylabels{isignal}));
            end
            if isignal == numel(signals)
                xlabel('Time [s]');
            end
        end
    end

    sgtitle(sprintf('Confronto segnali filtrati - Giunto %d', ijoint));
end

%% Parametro RMS di rumore
% Per ogni segnale calcolo:
%   residuo_rapido = segnale - movmean(segnale, finestra)
%   rumore_RMS = sqrt(mean(residuo_rapido.^2))
%
% La media mobile approssima il trend lento della traiettoria. Il residuo
% rapido contiene soprattutto rumore, vibrazioni e oscillazioni veloci.

noise_rms = zeros(n_cases, numel(signals), numel(joints_to_plot));

for icase = 1:n_cases
    n_window = max(3, round(trend_window_s * fs(icase)));

    idx = true(size(time{icase}));
    if ~isempty(time_window)
        idx = time{icase} >= time_window(1) & time{icase} <= time_window(2);
    end

    for isignal = 1:numel(signals)
        trend = movmean(signals{isignal}{icase}, n_window, 1, 'Endpoints', 'shrink');
        residual = signals{isignal}{icase} - trend;

        for j = 1:numel(joints_to_plot)
            ijoint = joints_to_plot(j);
            noise_rms(icase, isignal, j) = sqrt(mean(residual(idx, ijoint).^2));
        end
    end
end

figure('Name', 'Parametro RMS di rumore', 'Color', 'w');
tiledlayout(1, numel(signals), 'TileSpacing', 'compact', 'Padding', 'compact');

for isignal = 1:numel(signals)
    nexttile; grid on;
    values = squeeze(noise_rms(:, isignal, :)).';
    bar(joints_to_plot, values);
    xlabel('Giunto');
    ylabel('RMS residuo rapido');
    title(signal_names{isignal});
    xticks(joints_to_plot);

    if isignal == numel(signals)
        legend(case_labels, 'Location', 'best');
    end
end

sgtitle('Parametro RMS del rumore residuo');

fprintf('\nParametro RMS del residuo rapido nella finestra selezionata:\n');
for isignal = 1:numel(signals)
    fprintf('\n%s\n', signal_names{isignal});
    for icase = 1:n_cases
        fprintf('  %-24s', case_labels{icase});
        for j = 1:numel(joints_to_plot)
            fprintf(' J%d=%.4g', joints_to_plot(j), noise_rms(icase, isignal, j));
        end
        fprintf('\n');
    end
end
