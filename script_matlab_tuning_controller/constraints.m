function [cin, ceq] = constraints(controller_parameters, ...
    controller_frequency_response, ...
    process_frequency_response, ...
    desired_maximum_sensitivy, ...
    desired_phase_margin, ...
    low_freq_disturbance_attenuation, ...
    low_freq_threshold, ...
    high_freq_noise_attenuation, ...
    high_freq_threshold, ...
    frequency_vector)
% CONSTRAINTS  Evaluates inequality constraints for PI controller optimisation.
%
% Used by a constrained optimiser (e.g. fmincon) which requires constraints
% in the standard form:
%       cin(i)  <= 0   (inequality constraints, enforced here)
%       ceq(i)   = 0   (equality constraints, none used here)
%
% Each cin(i) is formulated as:  (actual_value - limit) <= 0
% so the constraint is satisfied when actual_value <= limit.
%
% -------------------------------------------------------------------------
% CONTROL DESIGN BACKGROUND
%
%  For a unity-feedback loop with plant G(jw) and controller C(jw):
%
%   Open-loop:          L(jw) = G(jw) * C(jw)
%   Sensitivity:        S(jw) = 1 / (1 + L(jw))     -- maps reference to error
%   Disturbance resp:   G(jw) / (1 + L(jw))          -- maps input dist. to output
%   Noise resp:         L(jw) / (1 + L(jw))          -- maps sensor noise to output
%
%  Good designs typically require:
%   1. Low |S| peak      --> robustness to plant uncertainty
%   2. Adequate phase margin --> stability margin
%   3. |G/(1+L)| small at low  frequencies --> reject slow disturbances
%   4. |L/(1+L)| small at high frequencies --> reject sensor noise
% -------------------------------------------------------------------------
%
% INPUTS:
%   controller_parameters            - vector [Ti, Kp] of tunable parameters
%   controller_frequency_response    - function handle: params -> @(w) C(jw)
%   process_frequency_response       - function handle: @(w) -> G(jw)
%   desired_maximum_sensitivy        - upper bound on max|S(jw)|
%                                      (typical value: 2  <=>  6 dB peak)
%   desired_phase_margin             - lower bound on phase margin [degrees]
%                                      (typical value: 30–60 deg)
%   low_freq_disturbance_attenuation - upper bound on |G/(1+L)| for w <= wl
%                                      (smaller = better disturbance rejection)
%   low_freq_threshold               - frequency wl [rad/s] below which
%                                      disturbance attenuation is enforced
%   high_freq_noise_attenuation      - upper bound on |L/(1+L)| for w >= wh
%                                      (smaller = less noise passed to output)
%   high_freq_threshold              - frequency wh [rad/s] above which
%                                      noise attenuation is enforced
%   frequency_vector                 - vector of frequencies [rad/s] for
%                                      numerical evaluation of all responses
%
% OUTPUTS:
%   cin  - 4-element vector of inequality constraints (feasible when all <= 0)
%   ceq  - empty (no equality constraints in this formulation)

% =========================================================================
% STEP 1 — Build frequency responses for current candidate parameters
% =========================================================================

% Evaluate the controller C(jw) using the current parameter candidate.
% controller is now a function handle @(w) -> complex vector.
controller = controller_frequency_response(controller_parameters);

% Form the open-loop response L(jw) = G(jw) * C(jw).
% All constraint calculations derive from L.
L = @(w) (process_frequency_response(w) .* controller(w));

% =========================================================================
% STEP 2 — Compute phase margin
% =========================================================================
% Phase margin (PM) measures how much additional phase lag the loop can
% tolerate before going unstable. It is measured AT the gain crossover
% frequency wc where |L(jwc)| = 1 (0 dB):
%
%   PM = 180° + angle(L(jwc))      [in degrees]
%
% PM > 0  --> stable closed loop
% PM > 30° is a common minimum requirement for adequate damping.

% Find wc: the frequency where |L(jw)| crosses 1 from above.
wc = find_cutting_frequency(L, frequency_vector);

if not(isempty(wc))
    % Insert wc exactly into the frequency grid so we can read the phase
    % precisely at that point (linear interpolation between grid points
    % would introduce error near the crossover).
    frequency_vector = sort([frequency_vector; wc]);
    i_wc = find(frequency_vector == wc, 1);

    % unwrap() removes 2π jumps from the phase curve, which could otherwise
    % cause a false phase reading if the phase crosses ±180° anywhere
    % below wc.
    L_phase = unwrap(angle(L(frequency_vector)));

    % Phase of L at the crossover frequency
    phase_wc = L_phase(i_wc);

    % Convert to phase margin in degrees.
    % At neutral stability, phase_wc = -π  =>  PM = 0°.
    % A positive PM means the phase has not yet reached -180°.
    actual_phase_margin = rad2deg(pi + phase_wc);
else
    % No crossover found: |L(jw)| < 1 for all w, meaning the loop gain
    % is always below 0 dB. This is unusual for a well-tuned controller,
    % so we assign inf to avoid falsely violating the PM constraint while
    % still allowing the optimiser to detect the real problem via fitness.
    actual_phase_margin = inf;
end

% =========================================================================
% STEP 3 — Define closed-loop performance functions
% =========================================================================

% Disturbance attenuation: how well the loop suppresses input disturbances.
% |G/(1+L)| should be SMALL at low frequencies where disturbances are large.
disturbance_attenuation = @(w) abs(process_frequency_response(w) ./ (1 + L(w)));

% Sensitivity function: maps reference tracking error.
% |S| = |1/(1+L)| should be SMALL overall; its peak (Ms) quantifies
% robustness — a large peak means the closed loop is sensitive to
% plant model errors (poor gain/phase margin).
sensitivity = @(w) abs(1 ./ (1 + L(w)));

% Complementary sensitivity (noise attenuation): fraction of sensor noise
% that reaches the plant input. Should be SMALL at high frequencies where
% sensor noise dominates.
noise_attenuation = @(w) abs(L(w) ./ (1 + L(w)));

% =========================================================================
% STEP 4 — Evaluate worst-case values over relevant frequency bands
% =========================================================================

% --- Low-frequency band: w <= low_freq_threshold ---
% Find the last index in frequency_vector that is at or below the threshold.
% 'last' gives the boundary point itself if it exists in the grid.
i_wl = find(frequency_vector <= low_freq_threshold, 1, 'last');

% Worst-case disturbance attenuation = maximum over [w_min, w_low_threshold].
% Using max (not mean) enforces the constraint at EVERY point in the band.
disturbance_attenuation_worst_case = max(disturbance_attenuation(frequency_vector(1:i_wl)));

% --- High-frequency band: w >= high_freq_threshold ---
i_wh = find(frequency_vector >= high_freq_threshold, 1, 'first');

% Worst-case noise attenuation = maximum over [w_high_threshold, w_max].
noise_attenuation_worst_case = max(noise_attenuation(frequency_vector(i_wh:end)));

% --- Sensitivity peak: evaluated over the full frequency range ---
actual_max_sensitivity = max(sensitivity(frequency_vector));

% =========================================================================
% STEP 5 — Assemble inequality constraints  cin(i) <= 0
% =========================================================================
% Each constraint is written as (actual - limit) <= 0, i.e. violated (> 0)
% when the actual value exceeds the designer-specified limit.

% cin(1): Disturbance rejection at low frequencies
%   Satisfied when disturbance_attenuation_worst_case <= low_freq_disturbance_attenuation
cin(1) = disturbance_attenuation_worst_case - low_freq_disturbance_attenuation;

% cin(2): Noise rejection at high frequencies
%   Satisfied when noise_attenuation_worst_case <= high_freq_noise_attenuation
cin(2) = noise_attenuation_worst_case - high_freq_noise_attenuation;

% cin(3): Sensitivity peak (robustness)
%   Satisfied when actual_max_sensitivity <= desired_maximum_sensitivy
cin(3) = actual_max_sensitivity - desired_maximum_sensitivy;

% cin(4): Phase margin (stability)
%   NOTE: inequality is REVERSED here because we want actual_phase_margin >= desired.
%   Rewriting: desired - actual <= 0  (satisfied when actual >= desired)
cin(4) = desired_phase_margin - actual_phase_margin;

% =========================================================================
% STEP 6 — NaN guard
% =========================================================================
% NaN values in cin cause fmincon to crash or behave unpredictably.
% This guard detects them, reports which constraint is affected (useful
% for debugging), and substitutes a large positive value so the optimiser
% treats the candidate as deeply infeasible and moves away.

errflag = 0;
if any(isnan(cin))
    errflag = 1;
    indnan = find(isnan(cin));
    for index = 1:length(indnan)
        fprintf('[Constraints] Element %d of cin is NaN!\n', indnan(index));
    end
end

% No equality constraints are used in this formulation.
ceq = [];

% Replace NaN-contaminated constraint vector with large positive values.
% Large positive cin(i) >> 0 signals strong infeasibility to the optimiser
% without providing meaningful gradient information (which would be wrong).
if errflag == 1
    warning('Some elements of cin and/or ceq are equal to NaN');
    cin = 1e5 * ones(size(cin));
end