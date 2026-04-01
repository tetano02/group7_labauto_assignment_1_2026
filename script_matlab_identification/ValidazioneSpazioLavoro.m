%% Validazione: modello in diversi punti di lavoro.
% Carico i risultati del test nei vari working point (wp)

clc;clear all;close all;
joint_number=1;
load(["model_joint"+joint_number])

model_name = 'gantry_portal_sea_soft';
folder_path = fullfile('..', model_name, 'tests');

tests=dir(fullfile(folder_path, "wp_validation_chirp_experiment_joint"+joint_number+"*.mat"));
for itest=1:length(tests)
    load([tests(itest).folder,filesep,tests(itest).name])
    fprintf('Giunto=%d\n',joint_number+1);
    fprintf('Chirp con Ampiezza %f da %f a %f\n',A,f0,f1)
    punto_di_lavoro=mean(joint_position);
    fprintf('Punto di lavoro [%f %f]\n',punto_di_lavoro(1),punto_di_lavoro(2))


    ngiunto=joint_number+1;
    Tc=time(2)-time(1);


    w0=2*pi*f0; %rad/s
    w1=2*pi*f1; %rad/s     Fs=1/Ts, Ws=2*pi/Ts, max w1 = 0.5*Ws=pi/Ts
    if w0>w1
        tmp=w0;
        w0=w1;
        w1=tmp;
    end
    control_action=joint_torque(:,ngiunto);
    output=joint_velocity(:,ngiunto);
    
    validation=iddata(output,control_action,Tc);
    freq_resp_validation = spafdr(validation);

    fr=freq_resp_validation.Frequency;
    for idx=1:length(fr)
        outside=(fr(idx)<w0) || (fr(idx)>w1);
        if outside
            freq_resp_validation.ResponseData(:,:,idx)=NaN;
        end
    end

    figure(1)
    subplot(3,1,1)
    plot(time,joint_position(:,ngiunto))
    xlabel('Time')
    ylabel('Position')
    hold on
    subplot(3,1,2)
    plot(time,output)
    xlabel('Time')
    ylabel('Velocity')
    hold on
    subplot(3,1,3)
    plot(time,control_action)
    xlabel('Time')
    ylabel('Torque')
    hold on
                
    figure(2)
    bode_opts = bodeoptions('cstprefs');
    bode_opts.PhaseWrapping = 'on';
    h = bodeplot(freq_resp_validation,'k', bode_opts);
    grid on
    hold on


end
figure(2)
hold on
bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';

bode(modello_continuo,bode_opts)

% carico le varie prove nei diversi spazi di lavoro e le plotto ->
% l'obiettivo è vedere quanto bene funziona il modello anche in altri punti