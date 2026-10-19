function hasil = analisis_step_test(csvFile, opts)
%ANALISIS_STEP_TEST  Dari CSV step test -> model FOPDT per step -> gain PI SIMC.
%
%   hasil = analisis_step_test('step_test_20261020.csv')
%   hasil = analisis_step_test(file, opts)
%
%   CSV minimal berisi 3 kolom (nama bisa diatur lewat opts):
%     opts.kol_t : waktu (s)                         ['elapsed_s']
%     opts.kol_u : PERINTAH bukaan katup (%)         ['servo_cmd']
%     opts.kol_y : suhu minyak (degC)                ['temperature']
%   Pakai perintah servo, bukan sudut terukur: sinyal terukur ber-noise dan
%   bisa memicu deteksi step palsu.
%
%   Opsi lain:
%     opts.Ts      : periode sampling controller di ESP32 (s)       [1]
%     opts.Ms_maks : batas robustness untuk rekomendasi             [1.7]
%     opts.plot    : true/false                                     [true]
%     opts.simpan  : simpan .mat + ringkasan .txt di sebelah CSV    [true]
%     opts.ident   : struct opsi untuk identifikasi_fopdt
%
%   Prosedur step test (ringkas):
%     1. Kontrol MANUAL (PID mati), katup di bukaan tetap, tunggu suhu steady
%        (drift < 0.3 degC/menit). Tutup panci seperti saat operasi normal.
%     2. Step bukaan +10..15 %, tahan sampai steady lagi (>= 4*tau, biasanya
%        20-40 menit). Lalu step berikutnya; sertakan juga step TURUN.
%     3. Ulangi di sekitar tiap setpoint kerja (mis. 150, 170, 180 degC).
%     4. Jangan masukkan bahan / ubah gas regulator selama tes.

if nargin < 2, opts = struct(); end
d = struct('kol_t', 'elapsed_s', 'kol_u', 'servo_cmd', 'kol_y', 'temperature', ...
    'Ts', 1, 'Ms_maks', 1.7, 'plot', true, 'simpan', true, 'ident', struct());
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end

T = readtable(csvFile, 'VariableNamingRule', 'preserve');
kol = {opts.kol_t, opts.kol_u, opts.kol_y};
hilang = kol(~ismember(kol, T.Properties.VariableNames));
if ~isempty(hilang)
    error('analisis_step_test:kolom', 'Kolom tidak ada di CSV: %s\nKolom tersedia: %s', ...
        strjoin(hilang, ', '), strjoin(T.Properties.VariableNames, ', '));
end
t = ke_angka(T.(opts.kol_t));
u = ke_angka(T.(opts.kol_u));
y = ke_angka(T.(opts.kol_y));

seg = identifikasi_fopdt(t, u, y, opts.ident);

% ---------- tuning per segmen ----------
baris = {};
baris{end+1} = sprintf('Analisis step test: %s', csvFile);
baris{end+1} = sprintf('%d step terdeteksi.\n', numel(seg));
baris{end+1} = sprintf('%-3s %-6s %7s %7s %8s %7s %7s %7s %7s  %s', ...
    'No', 'Arah', 'u0[%]', 'du[%]', 'Tss[C]', 'K', 'tau[s]', 'L[s]', 'NRMSE', 'Status');
for s = 1:numel(seg)
    r = seg(s);
    baris{end+1} = sprintf('%-3d %-6s %7.1f %+7.1f %8.1f %7.3f %7.1f %7.1f %6.1f%%  %s', ...
        r.no, r.arah, r.u0, r.du, r.y_akhir, r.K, r.tau, r.L, r.nrmse, ...
        ternary(r.valid, 'OK', 'PERIKSA')); %#ok<AGROW>
    for w = 1:numel(r.peringatan)
        baris{end+1} = sprintf('      ! %s', r.peringatan{w}); %#ok<AGROW>
    end
end

valid = find([seg.valid]);
if isempty(valid)
    baris{end+1} = sprintf(['\nTIDAK ADA segmen valid. Gain tidak dihitung -- perbaiki data ' ...
        'sesuai peringatan di atas.']);
    gain = []; rek = [];
else
    if numel(valid) < numel(seg)
        baris{end+1} = sprintf('\nSegmen berstatus PERIKSA tidak dipakai untuk tuning.');
    end
    % kandidat gain: SIMC Tc = 1,2,3 x L_eff untuk tiap model valid
    gain = [];
    for s = valid
        g = tuning_simc(seg(s).K, seg(s).tau, seg(s).L, [], struct('Ts', opts.Ts));
        for i = 1:numel(g)
            g(i).dari = s;
            % robustness TERBURUK terhadap SEMUA model valid (titik kerja lain)
            msAll = arrayfun(@(q) cek_robustness(seg(q).K, seg(q).tau, ...
                seg(q).L + opts.Ts/2, g(i).Kp, g(i).Ti, 0).Ms, valid);
            g(i).Ms_terburuk = max(msAll);
        end
        gain = [gain, g]; %#ok<AGROW>
    end

    baris{end+1} = sprintf('\nKandidat gain PI (SIMC). Ms_terburuk = diuji ke semua model valid.');
    baris{end+1} = sprintf('%-5s %7s %8s %8s %9s %6s %6s %6s %8s', ...
        'Model', 'Tc[s]', 'Kp', 'Ti[s]', 'Ki[1/s]', 'Ms', 'PM', 'GM', 'Ms_burk');
    for i = 1:numel(gain)
        q = gain(i);
        baris{end+1} = sprintf('%-5d %7.1f %8.3f %8.1f %9.5f %6.2f %6.1f %6.2f %8.2f', ...
            q.dari, q.Tc, q.Kp, q.Ti, q.Ki, q.Ms, q.PM, q.GM, q.Ms_terburuk); %#ok<AGROW>
    end

    % rekomendasi: aman di SEMUA titik kerja, lalu Ki terbesar
    % (Ki = Kp/Ti menentukan seberapa cepat gangguan beban/bahan masuk ditolak)
    aman = find([gain.Ms_terburuk] <= opts.Ms_maks);
    if isempty(aman)
        [~, ib] = min([gain.Ms_terburuk]);
        baris{end+1} = sprintf(['\n! Tidak ada kandidat dengan Ms_terburuk <= %.2f. Diambil yang ' ...
            'paling robust; plant sangat berbeda antar titik kerja -> pertimbangkan ' ...
            'gain scheduling per setpoint.'], opts.Ms_maks);
    else
        [~, j] = max([gain(aman).Ki]);
        ib = aman(j);
    end
    rek = gain(ib);
    baris{end+1} = sprintf(['\nREKOMENDASI PI tetap (berlaku di semua titik kerja yang diuji):\n' ...
        '  Kp0 = %.4f   Ki0 = %.5f   Kd0 = 0   (Ti = %.1f s, Tc = %.1f s, Ms_terburuk = %.2f)'], ...
        rek.Kp, rek.Ki, rek.Ti, rek.Tc, rek.Ms_terburuk);
    baris{end+1} = sprintf(['  Untuk build_deep_fryer_model.m: K/tau/L pakai model titik kerja utama, ' ...
        'Kp0/Ki0 di atas, Kd0 = 0.\n' ...
        '  Catatan adaptive: Kp = Kp0 + lambda_p*|e| MENAIKKAN gain di atas nilai ini. ' ...
        'Cek ulang Ms dengan Kp_maks/Ki_maks yang benar-benar bisa tercapai.']);
end

teks = strjoin(baris, newline);
fprintf('%s\n', teks);

hasil = struct('segmen', {seg}, 'gain', {gain}, 'rekomendasi', {rek}, 'ringkasan', teks, 'opts', opts);

[folder, nama] = fileparts(csvFile);
if opts.simpan
    save(fullfile(folder, [nama '_fopdt.mat']), 'hasil');
    fid = fopen(fullfile(folder, [nama '_fopdt.txt']), 'w');
    fprintf(fid, '%s\n', teks);
    fclose(fid);
end

if opts.plot
    fig = figure('Name', ['Step test ' nama], 'Color', 'w');
    n = numel(seg);
    for s = 1:n
        r = seg(s);
        ax1 = subplot(2, n, s);
        plot(r.t - r.t_step, r.y, '.', 'Color', [0.6 0.6 0.6]); hold on;
        plot(r.t - r.t_step, r.y_model, 'r', 'LineWidth', 1.5);
        grid on;
        title(sprintf('#%d %s: K=%.2f \\tau=%.0f L=%.0f', r.no, r.arah, r.K, r.tau, r.L));
        ylabel('Suhu (\circC)');
        if ~r.valid, title(ax1, [ax1.Title.String ' (PERIKSA)'], 'Color', [0.8 0 0]); end
        subplot(2, n, n + s);
        stairs(r.t - r.t_step, r.u, 'b'); grid on;
        xlabel('t - t_{step} (s)'); ylabel('Katup (%)');
    end
    legend(subplot(2, n, 1), {'Data', 'Model FOPDT'}, 'Location', 'best');
    if opts.simpan
        exportgraphics(fig, fullfile(folder, [nama '_fopdt.png']), 'Resolution', 150);
    end
end
end

function x = ke_angka(x)
if iscell(x) || isstring(x), x = str2double(x); end
x = double(x);
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
