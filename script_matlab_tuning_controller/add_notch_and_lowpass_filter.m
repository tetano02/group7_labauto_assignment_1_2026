function Plant = add_notch_and_lowpass_filter(sys_vel_motore, joint_number, lowpass_time_constant)
% ADD_NOTCH_AND_LOWPASS_FILTER Adds joint-specific notch and low-pass filters.
%
% Plant = add_notch_and_lowpass_filter(sys_vel_motore, joint_number, lowpass_time_constant)
%
% Inputs:
%   sys_vel_motore         - motor velocity plant for the selected joint
%   joint_number           - selected joint: 1, 2, or 3
%   lowpass_time_constant  - first-order low-pass time constant [s]
%
% Output:
%   Plant                  - filtered plant used for frequency-domain tuning

s = tf('s');

% Parameters aligned with gantry_portal_sea_soft/control_config.yaml.
switch joint_number
    case 1
        wn = 546;
        xci_z = 0.1;
        xci_p = 0.8;
    case 2
        wn = 662;
        xci_z = 0.2;
        xci_p = 0.8;
    case 3
        wn = 783;
        xci_z = 0.1;
        xci_p = 0.8;
    otherwise
        error('joint_number must be 1, 2, or 3.');
end

notch = (s^2 + 2*xci_z*wn*s + wn^2) / ...
        (s^2 + 2*xci_p*wn*s + wn^2);

lowpass = 1 / (lowpass_time_constant*s + 1);

Plant = sys_vel_motore * notch * lowpass;
end
