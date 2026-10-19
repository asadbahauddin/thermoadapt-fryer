function g = tuning_simc(K, tau, L, Tc, opts)
%TUNING_SIMC  Gain PI metode SIMC (Skogestad 2003) untuk model FOPDT + cek robustness.
%
%   g = tuning_simc(K, tau, L)          % Tc default = [1 2 3]*L_eff
%   g = tuning_simc(K, tau, L, Tc, opts)
%
%   SIMC (PI):  Kp = tau / (K*(Tc + L)),  Ti = min(tau, 4*(Tc + L)),  Td = 0
%   Tc = target konstanta waktu loop tertutup. Tc = L agresif, 2-3*L lebih
%   tahan terhadap kesalahan model.
%
%   Konversi ke bentuk controller di build_deep_fryer_model.m / firmware:
%     u = Kp*e + Ki*sum(e*dt) + Kd*de/dt   ->  Ki = Kp/Ti (1/s), Kd = Kp*Td (s)
%
%   opts.Ts : periode sampling controller (s) [1]. Setengahnya ditambahkan ke L
%             (efek zero-order hold digital).
%
%   Keluaran per Tc: Kp, Ti, Ki, Kd, dan robustness terhadap model itu sendiri:
%     Ms  : puncak fungsi sensitivitas (target <= 1.6, batas 2.0)
%     PM  : phase margin (deg, target >= 45)
%     GM  : gain margin (kali, target >= 2)

if nargin < 5, opts = struct(); end
if ~isfield(opts, 'Ts'), opts.Ts = 1; end
if K <= 0 || tau <= 0 || L < 0
    error('tuning_simc:model', 'Model tidak valid (K=%.3g, tau=%.3g, L=%.3g).', K, tau, L);
end
Le = L + opts.Ts/2;
if nargin < 4 || isempty(Tc), Tc = [1 2 3]*Le; end

g = struct('Tc', {}, 'Kp', {}, 'Ti', {}, 'Td', {}, 'Ki', {}, 'Kd', {}, ...
    'Ms', {}, 'PM', {}, 'GM', {});
for i = 1:numel(Tc)
    Kp = tau/(K*(Tc(i) + Le));
    Ti = min(tau, 4*(Tc(i) + Le));
    m = cek_robustness(K, tau, Le, Kp, Ti, 0);
    g(i) = struct('Tc', Tc(i), 'Kp', Kp, 'Ti', Ti, 'Td', 0, 'Ki', Kp/Ti, 'Kd', 0, ...
        'Ms', m.Ms, 'PM', m.PM, 'GM', m.GM);
end
end
