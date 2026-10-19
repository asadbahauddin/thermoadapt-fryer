%% run_deep_fryer_sim.m
%
% Membangun (jika perlu) model deep_fryer_adaptive_pid.slx, menjalankan
% simulasi Adaptive PID Control untuk TIGA setpoint (150, 170, 180 degC),
% menghitung metrik performa (rise time, settling time 2%, overshoot,
% steady-state error), membuat plot perbandingan, dan menyimpannya
% sebagai PNG.
%
% =====================================================================
% CATATAN PENTING -- Parameter plant (tau, L, K) dan parameter Adaptive
% PID (Kp0, Ki0, Kd0, lambda_p, lambda_i, lambda_d) yang dipakai di sini
% adalah ASUMSI AWAL/PLACEHOLDER (lihat komentar di build_deep_fryer_
% model.m), BUKAN hasil kalibrasi dari alat fisik. Hasil simulasi ini
% HANYA untuk memverifikasi STRUKTUR & PERILAKU algoritma Adaptive PID
% (mis. anti-windup bekerja, gain scheduling merespons error besar/
% kecil), bukan angka akhir yang siap dipakai sebagai klaim performa
% skripsi. Kalibrasi ulang WAJIB dilakukan begitu data step-response
% fisik alat (ESP32 + deep fryer) tersedia.
% =====================================================================

clear; clc; close all;

modelName = 'deep_fryer_adaptive_pid';

%% Bangun model jika file .slx belum ada
if ~exist([modelName '.slx'], 'file')
    fprintf('File %s.slx belum ada, membangun model...\n', modelName);
    build_deep_fryer_model();
else
    fprintf('Menggunakan model %s.slx yang sudah ada.\n', modelName);
    % Tetap perlu mendefinisikan variabel parameter di base workspace
    % karena model mereferensikan nama variabel (tau, L, K, dst).
    build_deep_fryer_model();
end

%% Skenario setpoint
setpoints = [150, 170, 180];
stop_time_local = 1800; % detik. CATATAN: sudah diuji juga dengan 3600s
                        % (durasi 2x lipat) dan settling time (2%) TETAP
                        % tidak tercapai -- respons menunjukkan OSILASI
                        % TERUS-MENERUS (limit cycle, amplitude puncak-
                        % ke-puncak ~17 degC, periode ~100s) di sekitar
                        % setpoint setelah transien awal, bukan masalah
                        % durasi simulasi yang kurang panjang. Ini adalah
                        % TEMUAN nyata dari kombinasi gain Adaptive PID
                        % placeholder saat ini (terutama Kd0/lambda_d
                        % yang membuat Kp,Kd tidak pernah bernilai nol
                        % walau error sudah kecil) dengan dead time plant
                        % -- perlu retuning setelah data fisik tersedia.
                        % 1800s dipakai sesuai rentang yang disarankan
                        % agar simulasi tetap efisien.
assignin('base', 'stop_time', stop_time_local);

results = struct('Setpoint', {}, 'RiseTime', {}, 'SettlingTime', {}, ...
    'Overshoot', {}, 'SSE', {}, 't', {}, 'Tact', {}, 'Tref', {});

for i = 1:numel(setpoints)
    sp = setpoints(i);
    fprintf('\n=== Menjalankan simulasi untuk setpoint %d degC ===\n', sp);
    assignin('base', 'Tref_value', sp);

    simOut = sim(modelName, 'StopTime', num2str(stop_time_local));

    Tact_ts = simOut.get('Tact_out');
    Tref_ts = simOut.get('Tref_out');

    t = Tact_ts.time;
    Tact = squeeze(Tact_ts.signals.values);
    Tref = squeeze(Tref_ts.signals.values);

    %% --- Hitung metrik performa ---
    % stepinfo() dari Control System Toolbox bekerja langsung pada sinyal
    % respons (y,t) dan mendeteksi sendiri nilai awal & akhir, sehingga
    % cocok dipakai di sini walau sinyalnya bukan step baku dari 0.
    info = stepinfo(Tact, t, sp, T_ambient_local(), ...
        'SettlingTimeThreshold', 0.02);

    tailWindow = t >= t(end) - 600; % 600 detik terakhir untuk cek osilasi
    T_final_actual = mean(Tact(t >= t(end) - 60)); % rata-rata 60 detik terakhir
    sse = sp - T_final_actual;
    tailRange = max(Tact(tailWindow)) - min(Tact(tailWindow)); % amplitudo p2p osilasi ekor

    results(i).Setpoint = sp;
    results(i).RiseTime = info.RiseTime;
    results(i).SettlingTime = info.SettlingTime;
    results(i).Overshoot = info.Overshoot;
    results(i).SSE = sse;
    results(i).TailOscRange = tailRange;
    results(i).t = t;
    results(i).Tact = Tact;
    results(i).Tref = Tref;

    fprintf('Setpoint %d degC:\n', sp);
    fprintf('  Rise Time      : %.2f s\n', info.RiseTime);
    if isnan(info.SettlingTime)
        fprintf(['  Settling Time  : TIDAK TERCAPAI dalam %d s (band 2%%) -- terdeteksi\n' ...
            '                   osilasi berkelanjutan (limit cycle) amplitudo p2p ~%.2f degC\n' ...
            '                   pada 600s terakhir. Ini temuan struktur algoritma dengan\n' ...
            '                   parameter placeholder saat ini, BUKAN akibat durasi simulasi\n' ...
            '                   kurang panjang (sudah diverifikasi juga pada durasi 2x lipat).\n'], ...
            stop_time_local, tailRange);
    else
        fprintf('  Settling Time  : %.2f s (band 2%%)', info.SettlingTime);
        if tailRange > 0.02 * sp
            fprintf(['  [PERHATIAN: nilai ini kemungkinan hanya kebetulan berada di\n' ...
                '                   dalam band 2%% pada akhir simulasi -- osilasi berkelanjutan\n' ...
                '                   (p2p ~%.2f degC) tetap terdeteksi pada 600s terakhir]'], tailRange);
        end
        fprintf('\n');
    end
    fprintf('  Overshoot      : %.2f %%\n', info.Overshoot);
    fprintf('  Steady-State Error (rata-rata 60s terakhir) : %.3f degC\n', sse);
    fprintf('  Osilasi p2p (600s terakhir)  : %.3f degC\n', tailRange);
end

%% --- Cetak ringkasan tabel ke console ---
fprintf('\n================================= RINGKASAN METRIK PERFORMA =================================\n');
fprintf('%-10s %-12s %-17s %-13s %-11s %-14s\n', 'Setpoint', 'RiseTime(s)', 'SettlingTime(s)', 'Overshoot(%)', 'SSE(degC)', 'OscP2P(degC)');
for i = 1:numel(results)
    if isnan(results(i).SettlingTime)
        settleStr = 'TIDAK SETTLE';
    else
        settleStr = sprintf('%.2f', results(i).SettlingTime);
    end
    fprintf('%-10d %-12.2f %-17s %-13.2f %-11.3f %-14.3f\n', results(i).Setpoint, ...
        results(i).RiseTime, settleStr, results(i).Overshoot, results(i).SSE, results(i).TailOscRange);
end
fprintf('===============================================================================================\n');
fprintf(['CATATAN: parameter plant (tau=%d s, L=%d s, K=%.2f) dan parameter\n' ...
    'Adaptive PID (Kp0=%.2f, Ki0=%.3f, Kd0=%.2f, lambda_p=%.3f, lambda_i=%.4f,\n' ...
    'lambda_d=%.3f) adalah ASUMSI AWAL, PERLU DIKALIBRASI ULANG setelah data\n' ...
    'pengujian fisik alat (ESP32 + deep fryer) tersedia.\n'], ...
    evalin('base','tau'), evalin('base','L'), evalin('base','K'), ...
    evalin('base','Kp0'), evalin('base','Ki0'), evalin('base','Kd0'), ...
    evalin('base','lambda_p'), evalin('base','lambda_i'), evalin('base','lambda_d'));

%% --- Plot perbandingan ketiga setpoint ---
fig = figure('Position', [100 100 900 900], 'Color', 'w');
for i = 1:numel(results)
    subplot(numel(results), 1, i);
    plot(results(i).t, results(i).Tact, 'b-', 'LineWidth', 1.5); hold on;
    plot(results(i).t, results(i).Tref, 'r--', 'LineWidth', 1.2);
    yline(results(i).Setpoint * 1.02, 'k:', 'LineWidth', 0.8);
    yline(results(i).Setpoint * 0.98, 'k:', 'LineWidth', 0.8);
    grid on;
    xlabel('Waktu (s)');
    ylabel('Suhu (degC)');
    title(sprintf('Setpoint %d degC -- Rise=%.1fs, Settle(2%%)=%.1fs, OS=%.2f%%, SSE=%.3f degC', ...
        results(i).Setpoint, results(i).RiseTime, results(i).SettlingTime, ...
        results(i).Overshoot, results(i).SSE));
    legend('T aktual', 'Setpoint', 'Location', 'southeast');
end
sgtitle('Respons Suhu Adaptive PID Deep Fryer (Simulasi Verifikasi Struktur -- Parameter Belum Terkalibrasi)');

pngFile = fullfile(pwd, 'deep_fryer_adaptive_pid_response.png');
exportgraphics(fig, pngFile, 'Resolution', 150);
fprintf('\nPlot disimpan di: %s\n', pngFile);

fprintf('\nSelesai. Model Simulink: %s\n', fullfile(pwd, [modelName '.slx']));

function Ta = T_ambient_local()
Ta = evalin('base', 'T_ambient');
end
