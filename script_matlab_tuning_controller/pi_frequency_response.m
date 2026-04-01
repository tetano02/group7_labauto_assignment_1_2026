function controller = pi_frequency_response(controller_parameters)
% PI_FREQUENCY_RESPONSE Returns a function handle that evaluates the
% frequency response of a PI controller at given frequencies.
%
% INPUT:
%   controller_parameters - a vector [Ti, Kp] where:
%       Ti = Integral time constant (controls how aggressively
%            the integrator acts; larger Ti = slower integral action)
%       Kp = Proportional gain (scales the overall controller output)
%
% OUTPUT:
%   controller - a function handle @(w) that accepts a vector of
%                frequencies (in rad/s) and returns the complex-valued
%                frequency response C(jw) at each frequency

% --- Extract tuning parameters from input vector ---
% The parameters are packed into a vector so they can be passed easily
% to optimisation routines (e.g. fminsearch, fmincon).
Kp = controller_parameters(2);   % Proportional gain
Ti = controller_parameters(1);   % Integral time constant

% --- Define the PI controller transfer function ---
% A PI controller in the time domain is:
%   u(t) = Kp * [ e(t) + (1/Ti) * integral(e(t)) ]
%
% Taking the Laplace transform gives the transfer function:
%   C(s) = Kp * (1 + 1/(Ti*s))
%        = Kp * (Ti*s + 1) / (Ti*s)
%
% In the frequency domain (s = jw), the controller:
%   - Provides proportional gain Kp at all frequencies
%   - Adds phase lead (improvement) at low frequencies via the integrator
%   - The integrator (1/s term) causes magnitude to roll off as w -> 0,
%     which eliminates steady-state error for step inputs

s = tf('s');           % Create the Laplace variable 's' as a tf object
C = Kp * (1 + 1/(Ti*s));   % Assemble the PI transfer function C(s)

% --- Build the frequency-response function handle ---
% freqresp(C, w) evaluates C(jw) at each frequency in vector w.
% It returns a 3D array of size [1 x 1 x length(w)], so reshape()
% converts it into a column vector of length(w) complex values,
% which is more convenient for plotting (Bode, Nyquist) or optimisation.
controller = @(w) reshape(freqresp(C, w), length(w), 1);