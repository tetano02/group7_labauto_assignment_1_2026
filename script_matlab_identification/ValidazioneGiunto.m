%% Validazione in presenza di limitazioni di velocità/posizione massima.
% In questo script vedremo la validazione con sweep in frequenza (chirp)
% 
% Carico i risultati del test

clc;clear all;close all;
joint_number=1;
load(["model_joint"+joint_number])

model_name = 'gantry_portal_sea_soft';
folder_path = fullfile('..', model_name, 'tests');

tests=dir(fullfile(folder_path, "validation_chirp_experiment_joint"+joint_number+"*.mat"));  

bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';

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
    
    figure
    h=bodeplot(freq_resp_validation,'k', bode_opts);
    grid on
    hold on
    showConfidence(h,3)
    bode(modello_continuo,bode_opts)
    drawnow
    xlim(sort([w0 w1]))
end

% Questo script prende tutte le prove e per tutte le prove plotta i dati e il
% modello