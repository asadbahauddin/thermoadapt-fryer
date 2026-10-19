function y = fopdt_sim(t, u, K, tau, L, y0, u0)
%FOPDT_SIM  Respon model FOPDT  G(s) = K/(tau*s+1)*exp(-L*s)  terhadap input u(t) sembarang.
%   y = fopdt_sim(t, u, K, tau, L, y0, u0)
%   t   : waktu (s), boleh tidak seragam, harus naik
%   u   : input (% bukaan katup) pada tiap t
%   y0  : keluaran awal (degC), u0 : input saat plant di steady state y0
%   Input sebelum t(1) dianggap u0. Hold orde nol antar sampel.

t = t(:); u = u(:);
n = numel(t);
ud = interp1(t, u, t - L, 'previous', u0);   % input tertunda L detik
ud(isnan(ud)) = u0;

y = zeros(n, 1);
y(1) = y0;
x = 0;                                        % deviasi keluaran dari y0
for k = 2:n
    a = exp(-(t(k) - t(k-1)) / tau);
    x = a*x + (1 - a)*K*(ud(k-1) - u0);
    y(k) = y0 + x;
end
end
