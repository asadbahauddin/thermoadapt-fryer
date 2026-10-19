function res = identifikasi_fopdt(t, u, y, opts)
%IDENTIFIKASI_FOPDT  Identifikasi K, tau, L dari data step test (open loop).
%
%   res = identifikasi_fopdt(t, u, y)            % deteksi step otomatis
%   res = identifikasi_fopdt(t, u, y, opts)
%
%   t : waktu (s), u : bukaan katup (%), y : suhu minyak (degC)
%   res : array struct, satu elemen per step yang terdeteksi.
%
%   Metode:
%   1. Estimasi awal metode dua titik (Smith, 28.3% & 63.2%).
%   2. Fitting least-squares model FOPDT terhadap SELURUH data segmen
%      (input apa adanya, jadi step yang tidak ideal / servo yang ramp tetap
%      dimodelkan benar), multi-start untuk menghindari minimum lokal di L.
%   3. Pemeriksaan kualitas data: steady sebelum step, sudah steady di akhir,
%      rasio sinyal/noise, dan kecocokan fit (NRMSE). Masalah dicatat di
%      res.peringatan -- JANGAN pakai hasil yang punya peringatan tanpa dicek.
%
%   opts (semua opsional):
%     min_du        : perubahan input minimum yang dianggap step (%)   [2]
%     gabung_s      : perubahan input berdekatan digabung jadi 1 step (s) [10]
%     pre_s         : jendela sebelum step untuk baseline (s)          [120]
%     post_s        : jendela akhir segmen untuk nilai akhir (s)       [120]
%     max_pre_slope : drift maks sebelum step (degC/menit)             [0.3]
%     min_snr       : |dy| minimum / std noise                         [10]
%     min_nrmse     : kecocokan fit minimum (%)                        [90]
%     t_step        : paksa waktu step manual (vektor, s) -- lewati deteksi otomatis
%     t_akhir       : batas akhir tiap segmen manual (vektor, s)

if nargin < 4, opts = struct(); end
opts = isi_default(opts, struct('min_du', 2, 'gabung_s', 10, 'pre_s', 120, ...
    'post_s', 120, 'max_pre_slope', 0.3, 'min_snr', 10, 'min_nrmse', 90, ...
    't_step', [], 't_akhir', []));

% ---------- validasi & bersihkan data ----------
t = double(t(:)); u = double(u(:)); y = double(y(:));
if ~(numel(t) == numel(u) && numel(u) == numel(y))
    error('identifikasi_fopdt:ukuran', 't, u, y harus sama panjang.');
end
ok = isfinite(t) & isfinite(u) & isfinite(y);
if any(~ok)
    warning('identifikasi_fopdt:nan', '%d baris berisi NaN/Inf dibuang.', sum(~ok));
end
t = t(ok); u = u(ok); y = y(ok);
[t, iu] = unique(t, 'stable');                % buang timestamp duplikat
u = u(iu); y = y(iu);
[t, is] = sort(t); u = u(is); y = y(is);
if numel(t) < 20
    error('identifikasi_fopdt:data', 'Data terlalu sedikit (%d sampel).', numel(t));
end
dts = diff(t);
if max(dts) > 10*median(dts)
    warning('identifikasi_fopdt:celah', ...
        'Ada celah data %.1f s (median sampling %.2f s). Cek koneksi logger.', max(dts), median(dts));
end

% ---------- segmentasi ----------
if isempty(opts.t_step)
    idx = find(abs(diff(u)) >= 0.5) + 1;         % semua perubahan input berarti
    tStep = [];
    k = 1;
    while k <= numel(idx)
        i0 = idx(k);
        j = k;
        while j < numel(idx) && t(idx(j+1)) - t(idx(j)) <= opts.gabung_s
            j = j + 1;
        end
        du = u(idx(j)) - u(i0 - 1);
        if abs(du) >= opts.min_du
            tStep(end+1) = t(i0 - 1); %#ok<AGROW>
        end
        k = j + 1;
    end
else
    tStep = opts.t_step(:).';
end
if isempty(tStep)
    error('identifikasi_fopdt:tanpa_step', ...
        'Tidak ada step input >= %.1f %% terdeteksi.', opts.min_du);
end
if isempty(opts.t_akhir)
    tAkhir = [tStep(2:end), t(end) + 1];
else
    tAkhir = opts.t_akhir(:).';
end

res = [];
for s = 1:numel(tStep)
    r = identifikasi_segmen(t, u, y, tStep(s), tAkhir(s), opts);
    r.no = s;
    res = [res, r]; %#ok<AGROW>
end
end

% =====================================================================
function r = identifikasi_segmen(t, u, y, ts, te, opts)
r = struct('no', 0, 't_step', ts, 'u0', NaN, 'du', NaN, 'y0', NaN, 'y_akhir', NaN, ...
    'K', NaN, 'tau', NaN, 'L', NaN, 'K_2titik', NaN, 'tau_2titik', NaN, 'L_2titik', NaN, ...
    'nrmse', NaN, 'rmse', NaN, 'noise_std', NaN, 'arah', '', 'peringatan', {{}}, ...
    'valid', false, 't', [], 'u', [], 'y', [], 'y_model', []);
warn = {};

iPre  = t >= ts - opts.pre_s & t < ts;
iSeg  = t >= ts - opts.pre_s & t < te;
if sum(iPre) < 5
    warn{end+1} = sprintf('Data sebelum step kurang dari %g s; baseline tidak andal.', opts.pre_s);
    iPre = t < ts & t >= t(find(t < ts, 1));
end
tP = t(iPre); yP = y(iPre);
u0 = median(u(iPre));
y0 = mean(yP);

% drift sebelum step
if numel(tP) >= 5
    p = polyfit(tP - tP(1), yP, 1);
    slope_pre = p(1)*60;
    noise = std(yP - polyval(p, tP - tP(1)));
else
    slope_pre = NaN; noise = NaN;
end
if abs(slope_pre) > opts.max_pre_slope
    warn{end+1} = sprintf(['Suhu belum steady sebelum step (drift %.2f degC/menit > %.2f). ' ...
        'Tunggu lebih lama sebelum memberi step.'], slope_pre, opts.max_pre_slope);
end

tS = t(iSeg); uS = u(iSeg); yS = y(iSeg);
iPost = tS >= tS(end) - opts.post_s;
uAkhir = median(uS(iPost));
yAkhir = mean(yS(iPost));
du = uAkhir - u0;
dy = yAkhir - y0;
r.u0 = u0; r.du = du; r.y0 = y0; r.y_akhir = yAkhir; r.noise_std = noise;
r.arah = ternary(du > 0, 'naik', 'turun');

if abs(du) < opts.min_du
    warn{end+1} = sprintf('Step input terlalu kecil (%.2f %%).', du);
    r.peringatan = warn; r.t = tS; r.u = uS; r.y = yS;
    return;
end
if isfinite(noise) && noise > 0 && abs(dy) < opts.min_snr*noise
    warn{end+1} = sprintf(['Perubahan suhu %.2f degC terlalu kecil dibanding noise ' ...
        '(std %.2f degC). Perbesar step.'], dy, noise);
end

% ---------- estimasi dua titik (Smith) ----------
yf = movmedian(yS, 9);
iAfter = find(tS >= ts);
t28 = waktu_lewat(tS(iAfter), yf(iAfter), y0 + 0.283*dy, dy);
t63 = waktu_lewat(tS(iAfter), yf(iAfter), y0 + 0.632*dy, dy);
if isfinite(t28) && isfinite(t63) && t63 > t28
    tau2 = 1.5*(t63 - t28);
    L2 = max(t63 - tau2 - ts, 0);
else
    tau2 = max((tS(end) - ts)/4, 1);
    L2 = 0;
    warn{end+1} = 'Titik 28.3%/63.2% tidak ditemukan; estimasi awal kasar dipakai.';
end
K2 = dy/du;
r.K_2titik = K2; r.tau_2titik = tau2; r.L_2titik = L2;

% ---------- fitting least squares (multi-start) ----------
% parameter: [K, log(tau), sqrt(L), y0] -> tau>0 dan L>=0 otomatis
cost = @(p) sum((yS - fopdt_sim(tS, uS, p(1), exp(p(2)), p(3)^2, p(4), u0)).^2);
fo = optimset('Display', 'off', 'MaxFunEvals', 4000, 'MaxIter', 4000, 'TolX', 1e-6, 'TolFun', 1e-8);
best = inf; pBest = [];
for Lm = [0, 0.5, 1, 2]
    for tm = [0.7, 1, 1.5]
        p0 = [K2, log(max(tau2*tm, 1)), sqrt(max(L2*Lm, 0.5)), y0];
        [p, f] = fminsearch(cost, p0, fo);
        if f < best, best = f; pBest = p; end
    end
end
r.K = pBest(1); r.tau = exp(pBest(2)); r.L = pBest(3)^2;
yM = fopdt_sim(tS, uS, r.K, r.tau, r.L, pBest(4), u0);
r.rmse = sqrt(mean((yS - yM).^2));
r.nrmse = 100*(1 - norm(yS - yM)/norm(yS - mean(yS)));
r.t = tS; r.u = uS; r.y = yS; r.y_model = yM;

% ---------- pemeriksaan hasil ----------
durasi = tS(end) - ts;
if durasi < 4*r.tau + r.L
    warn{end+1} = sprintf(['Segmen hanya %.0f s setelah step, < 4*tau+L = %.0f s. ' ...
        'Suhu belum steady, K bisa meleset. Tahan step lebih lama.'], durasi, 4*r.tau + r.L);
end
iP = tS >= tS(end) - opts.post_s;
if sum(iP) >= 5
    pp = polyfit(tS(iP) - tS(find(iP, 1)), yS(iP), 1);
    if abs(pp(1)*r.tau) > 0.05*abs(dy)
        warn{end+1} = sprintf('Suhu masih bergerak di akhir segmen (%.2f degC/menit).', pp(1)*60);
    end
end
if r.nrmse < opts.min_nrmse
    warn{end+1} = sprintf(['Kecocokan model rendah (NRMSE %.1f %% < %.0f %%). ' ...
        'Cek gangguan (tutup/bahan masuk/angin) selama tes.'], r.nrmse, opts.min_nrmse);
end
if sign(r.K) ~= sign(K2) || r.K <= 0
    warn{end+1} = 'Tanda gain tidak wajar (suhu harus naik bila katup dibuka).';
end
if abs(r.K - K2) > 0.25*abs(K2)
    warn{end+1} = sprintf('K fitting (%.3f) jauh dari K dua titik (%.3f).', r.K, K2);
end
r.peringatan = warn;
r.valid = isempty(warn);
end

% =====================================================================
function tc = waktu_lewat(t, y, lvl, dy)
% waktu pertama y melewati lvl (searah dy), interpolasi linier
if dy >= 0, i = find(y >= lvl, 1); else, i = find(y <= lvl, 1); end
if isempty(i), tc = NaN; return; end
if i == 1, tc = t(1); return; end
tc = t(i-1) + (lvl - y(i-1))*(t(i) - t(i-1))/(y(i) - y(i-1));
end

function o = isi_default(o, d)
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
