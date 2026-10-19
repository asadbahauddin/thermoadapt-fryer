function m = cek_robustness(K, tau, L, Kp, Ti, Td)
%CEK_ROBUSTNESS  Ms, PM, GM loop PID (bentuk paralel) + plant FOPDT.
%   m = cek_robustness(K, tau, L, Kp, Ti, Td)
%   Dipakai juga untuk menguji satu set gain terhadap model titik kerja LAIN.
%   Ms <= 1.6 baik, <= 2.0 masih bisa diterima, > 2.0 rawan osilasi.

w = logspace(log10(1e-3/(tau + L)), log10(1e2/max(L, 1e-2)), 40000);
s = 1j*w;
G = K*exp(-L*s)./(tau*s + 1);
C = Kp*(1 + 1./(Ti*s) + Td*s);
Lw = C.*G;
m.Ms = max(1./abs(1 + Lw));
ph = unwrap(angle(Lw))*180/pi;
mag = abs(Lw);
ic = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
if isempty(ic), m.PM = Inf; else, m.PM = 180 + ph(ic); end
ip = find(ph(1:end-1) > -180 & ph(2:end) <= -180, 1);
if isempty(ip), m.GM = Inf; else, m.GM = 1/mag(ip); end
end
