function J = fitness(controller_parameters, ...
    controller_frequency_response, ...
    process_frequency_response, desired_wc, frequency_vector)
% FITNESS  Objective function for PI controller tuning via frequency design.
%
% This function is intended to be minimised by an optimisation routine
% (e.g. fminsearch). It tunes the controller so that the open-loop
% crossover frequency matches a desired target value.
%
% -------------------------------------------------------------------------
% WHAT IS THE CROSSOVER FREQUENCY (wc)?
%   The gain crossover frequency is the frequency where the open-loop
%   magnitude |L(jw)| = 1  (i.e. 0 dB).
%   It is a key indicator of closed-loop bandwidth and response speed:
%       - Higher wc  -->  faster closed-loop response
%       - Lower  wc  -->  slower, more conservative response
%   By driving actual_wc --> desired_wc, we shape the closed-loop behaviour.
% -------------------------------------------------------------------------
%
% INPUTS:
%   controller_parameters        - vector of tunable parameters [Ti, Kp]
%                                  passed to the controller model
%   controller_frequency_response - function handle @(params) -> @(w)
%                                  builds the controller C(jw) from params
%   process_frequency_response   - function handle @(w) returning the
%                                  complex frequency response G(jw) of the
%                                  plant (fixed, not tuned)
%   desired_wc                   - scalar, target crossover frequency [rad/s]
%   frequency_vector             - vector of frequencies [rad/s] over which
%                                  the open-loop response is evaluated
%
% OUTPUT:
%   J  - scalar cost (lower is better). Defined as:
%           J = (actual_wc - desired_wc)^2   if a crossover is found
%           J = 1e6                           if no crossover exists
%        Squaring the error makes J always non-negative and penalises
%        large deviations more heavily than small ones.

% --- Step 1: Evaluate the controller frequency response ---
% controller_frequency_response is a function that takes the parameter
% vector and returns a function handle @(w) for C(jw).
% Calling it here "bakes in" the current candidate parameters.
controller = controller_frequency_response(controller_parameters);

% --- Step 2: Form the open-loop transfer function L(jw) = G(jw) * C(jw) ---
% L is the product of the plant and controller responses.
% In frequency-domain design, stability and performance are analysed
% through L(jw):
%   - |L(jw)| tells us gain (crossover frequency lives here)
%   - angle(L(jw)) tells us phase (phase margin lives here)
% The .* operator multiplies element-wise across all frequencies in w.
L = @(w) (process_frequency_response(w) .* controller(w));

% --- Step 3: Find the actual crossover frequency ---
% find_cutting_frequency searches frequency_vector for the frequency
% where |L(jw)| crosses 1 (0 dB) from above.
% It returns [] if no such crossing exists (e.g. the loop gain never
% reaches 1, meaning the system is very sluggish or unstable in gain).
actual_wc = find_cutting_frequency(L, frequency_vector);

% --- Step 4: Compute the cost J ---
if not(isempty(actual_wc))
    % Normal case: a crossover was found.
    % Minimising (actual_wc - desired_wc)^2 drives the crossover
    % towards the target frequency. This is a least-squares cost.
    J = (actual_wc - desired_wc)^2;
else
    % Penalty case: no crossover found with these parameters.
    % Returning a large constant penalises the optimiser for exploring
    % regions where the open-loop gain never reaches 0 dB.
    % This steers the search back towards feasible parameter values.
    J = 1000000;
end