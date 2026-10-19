function lulus = uji_sintetis(folderOut)
%UJI_SINTETIS  Uji toolkit step test TANPA hardware.
%   Membuat data step test palsu dengan parameter FOPDT yang DIKETAHUI
%   (beda tiap titik kerja), ditambah realita lapangan: noise sensor,
%   kuantisasi 0.25 degC (MAX31855), jitter sampling, baris NaN, servo ramp.
%   Lalu cek apakah identifikasi_fopdt menemukan kembali parameternya, dan
%   apakah segmen yang belum steady ditandai PERIKSA.
%
%   lulus = uji_sintetis()            % CSV & plot ke tempdir
%   lulus = uji_sintetis(folderOut)

if nargin < 1, folderOut = tempdir; end
rng(42);

% ---------- plant "sebenarnya" per segmen ----------
%            u_awal  u_akhir  K     tau   L    durasi(s)
skenario = [ 30      45       2.0   300   25   2400;    % naik, sekitar 150 C
             45      60       1.6   260   20   2400;    % naik, sekitar 175 C (gain turun)
             60      45       1.3   380   20   2400;    % turun (pendinginan lebih lambat)
             45      55       1.6   260   20    600];   % SENGAJA terlalu pendek -> harus PERIKSA
Ta = 30;
Tpre = 900;                                         % steady awal

t = []; u = []; y = [];
tNow = 0; yNow = Ta + skenario(1,3)*skenario(1,1);  % mulai steady
uPrev = skenario(1,1);
for s = 1:size(skenario, 1)
    dur = skenario(s, 6) + (s == 1)*Tpre;
    ts = tNow + (s == 1)*Tpre;                       % waktu step segmen ini
    tt = (tNow:1:tNow + dur - 1)' + 0.15*randn(dur, 1);   % jitter sampling
    tt(1) = tNow;
    tt = sort(tt);
    uu = uPrev*ones(size(tt));
    ramp = tt >= ts;
    uu(ramp) = skenario(s, 2);
    iR = find(ramp, 3);                              % servo ramp 3 sampel
    uu(iR) = uPrev + (skenario(s, 2) - uPrev)*[1/3; 2/3; 1];
    % simulasi dengan parameter segmen ini, lanjut dari keadaan sebelumnya
    yy = fopdt_sim([tNow - 1; tt], [uPrev; uu], skenario(s,3), skenario(s,4), ...
        skenario(s,5), yNow, uPrev);
    yy = yy(2:end);
    t = [t; tt]; u = [u; uu]; y = [y; yy]; %#ok<AGROW>
    tNow = tt(end) + 1; yNow = yy(end); uPrev = skenario(s, 2);
end
ySens = round((y + 0.15*randn(size(y)))/0.25)*0.25; % noise + kuantisasi MAX31855
ySens([500 3100 6000]) = NaN;                        % paket hilang

T = table(t, u, ySens, 'VariableNames', {'elapsed_s', 'servo_target_pct', 'suhu_c'});
csv = fullfile(folderOut, 'uji_sintetis_step.csv');
writetable(T, csv);

hasil = analisis_step_test(csv, struct('plot', true, 'simpan', true));
seg = hasil.segmen;

% ---------- verifikasi ----------
fprintf('\n===== VERIFIKASI terhadap parameter sebenarnya =====\n');
lulus = numel(seg) == size(skenario, 1);
if ~lulus
    fprintf('GAGAL: %d segmen terdeteksi, seharusnya %d.\n', numel(seg), size(skenario, 1));
    return;
end
for s = 1:3
    eK = abs(seg(s).K/skenario(s,3) - 1);
    eT = abs(seg(s).tau/skenario(s,4) - 1);
    eL = abs(seg(s).L - skenario(s,5));
    ok = eK < 0.05 && eT < 0.10 && eL < 5 && seg(s).valid;
    lulus = lulus && ok;
    fprintf('#%d K %.3f (benar %.2f, %4.1f%%)  tau %.1f (benar %d, %4.1f%%)  L %.1f (benar %d)  valid=%d  -> %s\n', ...
        s, seg(s).K, skenario(s,3), 100*eK, seg(s).tau, skenario(s,4), 100*eT, ...
        seg(s).L, skenario(s,5), seg(s).valid, ternary(ok, 'LULUS', 'GAGAL'));
end
ok4 = ~seg(4).valid;
lulus = lulus && ok4;
fprintf('#4 segmen terlalu pendek ditandai PERIKSA: %s\n', ternary(ok4, 'LULUS', 'GAGAL'));
ok5 = ~isempty(hasil.rekomendasi) && hasil.rekomendasi.Ms_terburuk <= 1.7;
lulus = lulus && ok5;
fprintf('Rekomendasi gain ada dan Ms_terburuk <= 1.7: %s\n', ternary(ok5, 'LULUS', 'GAGAL'));
fprintf('HASIL AKHIR: %s\n', ternary(lulus, 'SEMUA LULUS', 'ADA YANG GAGAL'));
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
