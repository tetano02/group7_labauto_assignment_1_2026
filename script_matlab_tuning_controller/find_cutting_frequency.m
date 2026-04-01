function wc = find_cutting_frequency(frequency_response, frequency_vector)
% FIND_CUTTING_FREQUENCY  Finds the gain crossover frequency of a system.
%
% The gain crossover frequency wc is defined as the frequency [rad/s] at
% which the open-loop magnitude |L(jwc)| = 1  (equivalently, 0 dB).
%
% It is a critical design parameter:
%   - It sets the closed-loop bandwidth (speed of response)
%   - Phase margin is measured at this exact frequency
%   - The fitness function drives wc towards a designer-specified target
%
% The algorithm works in three stages:
%   1. Quick scan  — evaluate |L(jw)| on the supplied frequency grid to
%                    check whether a 0 dB crossing is visible
%   2. Bracket     — if the grid misses the crossing, search outside the
%                    grid by expanding wc geometrically (×10 or ÷10)
%   3. Refine      — call fzero() to pin down wc to machine precision
%
% INPUTS:
%   frequency_response  - function handle @(w) returning the complex
%                         frequency response L(jw) at each frequency in w
%   frequency_vector    - vector of frequencies [rad/s] used for the
%                         initial coarse scan (logarithmically spaced is
%                         typical, e.g. logspace(-3, 3, 1000))
%
% OUTPUT:
%   wc  - scalar crossover frequency [rad/s], or [] if none exists

% =========================================================================
% STAGE 1 — Coarse scan over the supplied frequency grid
% =========================================================================
% Evaluate the magnitude of L(jw) at every grid point in one vectorised
% call. This is cheap and immediately tells us three things:
%   (a) magnitude crosses 1 somewhere inside the grid  --> normal case
%   (b) magnitude is always < 1 on the grid           --> may cross below
%   (c) magnitude is always > 1 on the grid           --> may cross above
magnitude = abs(frequency_response(frequency_vector));

if ~isempty(find(magnitude > 1, 1)) && ~isempty(find(magnitude < 1, 1))

    % -----------------------------------------------------------------
    % CASE (a): The 0 dB crossing is bracketed within the frequency grid.
    % -----------------------------------------------------------------
    % Find the first index where the magnitude drops below 1.
    % This is where the curve crosses 0 dB from above (the typical,
    % minimum-phase crossing we care about for phase margin).
    % Using 'first' ensures we find the lowest-frequency crossing when
    % there are multiple (e.g. non-minimum phase systems).
    i_wc = find(magnitude < 1, 1, 'first');

    if isempty(i_wc)
        % Guard: logically impossible given the outer if-condition,
        % but left to catch any future code restructuring.
        error("This should not happen");
    end

    % Use the grid point just below the crossing as the starting bracket
    % for fzero (Stage 3 will refine this to exact precision).
    wc = frequency_vector(i_wc);

else

    % -----------------------------------------------------------------
    % CASES (b) and (c): The grid does not contain a 0 dB crossing.
    % We must search outside the grid before refining.
    % -----------------------------------------------------------------

    if abs(magnitude(1)) < 1

        % -------------------------------------------------------------
        % CASE (b): |L(jw)| < 1 everywhere on the grid.
        % The crossing, if it exists, must be at a LOWER frequency than
        % the start of the grid (the system may have very low bandwidth).
        % Strategy: start at a very small frequency and multiply by 10
        % until the magnitude exceeds 1 — i.e. we cross from below.
        % -------------------------------------------------------------
        wc = 1e-80;   % Start far below any physically meaningful frequency

        % Sanity check: if even at near-DC the magnitude is below 1,
        % the loop gain never reaches 0 dB at any frequency.
        % This happens when the plant+controller gain is always < 1
        % (e.g. a heavily attenuated system). Return [] to signal
        % "no crossover" to the caller (fitness assigns a penalty).
        if abs(frequency_response(wc)) < 1
            warning("L(s) does not cut the 0dB");
            wc = [];
            return
        end

        % Walk upward in decade steps until the magnitude falls below 1.
        % After this loop, wc is the first decade point where |L| < 1,
        % so the true crossing lies in [wc/10, wc].
        % fzero (Stage 3) will refine within that bracket.
        while abs(frequency_response(wc)) > 1
            wc = wc * 10;
        end

    else

        % -------------------------------------------------------------
        % CASE (c): |L(jw)| > 1 everywhere on the grid.
        % The crossing, if it exists, must be at a HIGHER frequency than
        % the end of the grid (the system may have very high bandwidth,
        % or may not roll off — i.e. it is not strictly proper).
        %
        % A strictly proper transfer function (more poles than zeros)
        % MUST roll off to 0 as w → ∞, guaranteeing a 0 dB crossing.
        % An improper system keeps |L| > 1 forever (not realisable in
        % practice, but possible in a numerical model).
        % Strategy: start at a very large frequency and divide by 10
        % until the magnitude drops below 1 — i.e. we cross from above.
        % -------------------------------------------------------------
        wc = 1e80;    % Start far above any physically meaningful frequency

        % Sanity check: if even at near-infinite frequency the magnitude
        % is still above 1, the system does not roll off. Warn and return.
        if abs(frequency_response(wc)) > 1
            warning("L(s) is not strictly proper");
            wc = [];
            return
        end

        % Walk downward in decade steps until the magnitude exceeds 1.
        % After this loop, wc is the first decade point where |L| > 1,
        % so the true crossing lies in [wc, wc*10].
        while abs(frequency_response(wc)) < 1
            wc = wc / 10;
        end

    end
end

% =========================================================================
% STAGE 3 — Refine with fzero
% =========================================================================
% fzero finds the root of a scalar equation f(w) = 0.
% We define f(w) = |L(jw)| - 1, so its root is precisely the 0 dB crossing.
%
% The wc value from Stages 1-2 provides a good starting point (not a full
% bracket, but fzero can work from a single point using a finite-difference
% secant method internally).
%
% Why fzero and not bisection?
%   fzero uses a combination of bisection, secant, and inverse quadratic
%   interpolation — it converges superlinearly and is robust to flat regions
%   near the root, where pure secant fails.
wc = fzero(@(w) abs(frequency_response(w)) - 1, wc);

% =========================================================================
% SIGN GUARD
% =========================================================================
% fzero operates on a real scalar w and can, in rare edge cases, wander
% into negative w territory if the starting point is very close to 0 and
% the function is nearly flat. Negative frequency has no physical meaning.
% If this happens, reflect the result and re-solve from the positive side.
if wc < 0
    wc = fzero(@(w) abs(frequency_response(w)) - 1, -wc);
end