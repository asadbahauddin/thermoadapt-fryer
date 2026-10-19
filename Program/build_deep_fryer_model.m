%% build_deep_fryer_model.m
%
% Membangun model Simulink (.slx) untuk simulasi Adaptive PID Control
% pada sistem Deep Fryer (dinamika termal minyak) secara TERPROGRAM.
%
% Skripsi: "Rekayasa Sistem Deep Fryer Cerdas Berbasis Adaptive PID
% Control untuk Optimasi Stabilitas Suhu dan Kualitas Penggorengan
% Produk Olahan Ubi"
%
% =====================================================================
% CATATAN PENTING -- SEMUA PARAMETER NUMERIK DI BAWAH INI ADALAH ASUMSI
% AWAL / PLACEHOLDER untuk keperluan VERIFIKASI STRUKTUR SIMULASI.
% Nilai-nilai ini BUKAN hasil kalibrasi dari alat fisik (ESP32 + deep
% fryer nyata). Begitu data pengujian fisik alat tersedia (respons suhu
% aktual terhadap bukaan katup gas), parameter tau, L, K (plant) dan
% Kp0, Ki0, Kd0, lambda_p, lambda_i, lambda_d (controller) WAJIB
% dikalibrasi ulang (misal via System Identification Toolbox / manual
% curve fitting terhadap data step-response nyata alat).
% =====================================================================

function build_deep_fryer_model()

modelName = 'deep_fryer_adaptive_pid';

%% ---------------------------------------------------------------
%% 1. PARAMETER PLANT (dinamika termal minyak) -- ASUMSI/PLACEHOLDER
%% ---------------------------------------------------------------
% Model pendekatan generik: First-Order + Dead Time (FOPDT)
%   G(s) = K / (tau*s + 1) * exp(-L*s)
%
% tau, L, K berikut adalah TEBAKAN AWAL, bukan hasil identifikasi
% sistem dari alat fisik. Volume minyak diasumsikan ~6 liter (masih
% perlu dikonfirmasi ke pembimbing) -- tidak dipakai eksplisit karena
% model first-order sederhana ini tidak memodelkan volume/massa secara
% langsung, hanya menangkap perilaku "lag" dominan pada skala waktu
% tersebut.
tau = 180;   % time constant (detik) -- ASUMSI, perlu dikalibrasi ulang
L   = 10;    % dead time (detik) -- ASUMSI, perlu dikalibrasi ulang
K   = 1.9;   % steady-state gain (degC/% bukaan katup) -- ASUMSI
T_ambient = 30;  % suhu awal / ambient (degC)
% oil_volume_liter_NOTE = 6 -- asumsi volume minyak, TIDAK dipakai dalam persamaan

%% ---------------------------------------------------------------
%% 2. PARAMETER CONTROLLER -- Adaptive PID (gain scheduling)
%% ---------------------------------------------------------------
% Hukum adaptasi:
%   e(k)    = Tref - Tact(k)
%   Kp(k)   = Kp0 + lambda_p*|e(k)|
%   Ki(k)   = Ki0 + lambda_i*|e(k)|      <-- lihat FIX #2 di bagian 7:
%                                            spesifikasi awal skripsi
%                                            memakai |sum_e| di sini,
%                                            tapi TERBUKTI (via simulasi
%                                            isolasi) menyebabkan limit
%                                            cycle/tidak pernah settle,
%                                            karena Ki jadi ikut membesar
%                                            seiring sum_e yang memang
%                                            dirancang untuk terus
%                                            tumbuh -- umpan balik
%                                            positif. Dijadwalkan ulang
%                                            dari |e(k)|, konsisten
%                                            dengan skema Kp yang sudah
%                                            terbukti aman.
%   Kd(k)   = Kd0 + lambda_d*|d_filt(k)|  <-- lihat FIX #4 di bagian 7:
%                                            versi sebelumnya memakai
%                                            |delta_e| (error mentah per
%                                            sample) untuk MENJADWALKAN
%                                            Kd, padahal delta_e/dt (yang
%                                            sudah difilter jadi d_filt)
%                                            yang dipakai di TERM output.
%                                            Ini scaling mismatch laten:
%                                            delta_e bergantung pada dt
%                                            (sample time), jadi kalau dt
%                                            berubah (mis. ESP32 real
%                                            tidak jalan tepat 1s), gain
%                                            Kd ikut salah skala walau
%                                            laju perubahan fisik error
%                                            sebenarnya sama. Saat ini
%                                            dt=1s tetap sehingga bug ini
%                                            "tidur" (delta_e/dt=delta_e
%                                            secara numerik), tapi
%                                            berbahaya kalau dt berubah.
%                                            Dijadwalkan dari d_filt
%                                            (sudah benar unit degC/s DAN
%                                            sudah difilter, jadi lebih
%                                            tahan noise sensor).
%   u(k)    = Kp(k)*e(k) + Ki(k)*sum_e + Kd(k)*d_filt(k) [delta_e/dt difilter, lihat FIX #1]
%
% Nilai awal berikut adalah ASUMSI untuk verifikasi struktur simulasi,
% bukan hasil tuning akhir. Disimpan sebagai variabel workspace supaya
% mudah diubah tanpa mengedit block.
Kp0 = 2;
Ki0 = 0.05;
Kd0 = 0.5;
lambda_p = 0.02;
lambda_i = 0.001;
lambda_d = 0.01;

% --- FIX #5: batas maksimum Kp/Ki/Kd (safety clamp) --------------------
% Sebelumnya TIDAK ADA batas atas untuk gain adaptif -- kalau suatu saat
% lambda_p/lambda_i/lambda_d di-tuning ulang jadi lebih agresif (atau
% error jadi sangat besar akibat sensor gagal/kabel lepas dsb.), gain
% bisa membesar tanpa batas dan menghasilkan aksi kontrol yang tidak
% aman. Batas berikut adalah jaring pengaman (safety net) -- dengan nilai
% lambda saat ini, batas ini TIDAK PERNAH benar-benar tercapai dalam
% kondisi normal (Kp maksimum secara teoritis pada error 150 degC =
% Kp0+lambda_p*150 = 2+3 = 5, jauh di bawah Kp_max=10 dst.), jadi aman
% ditambahkan tanpa mengubah perilaku simulasi saat ini.
Kp_max = 10;
Ki_max = 1;
Kd_max = 5;

% --- TAMBAHAN: derivative filter + error deadband -----------------------
% Simulasi awal (sebelum penambahan ini) menunjukkan sistem TIDAK PERNAH
% settle di ketiga setpoint -- terjadi limit cycle (osilasi berkelanjutan
% p2p ~11-17 degC, periode ~100s) setelah transien awal. Berdasarkan
% riset, dua penyebab paling umum untuk pola ini pada kontrol suhu PID
% adalah: (1) derivative term tidak difilter (referensi: "Improving the
% Beginner's PID - Derivative Kick", brettbeauregard.com), dan (2) tidak
% ada deadband di sekitar setpoint (referensi: ESPHome PID Climate docs,
% Powerblanket PID Temperature Controller Guide). Akar masalah UTAMA yang
% ditemukan lewat simulasi isolasi ternyata scheduling Ki berbasis
% |sum_e| (lihat FIX #2 di atas) -- derivative filter & deadband di sini
% tetap dipertahankan sebagai lapis pencegahan tambahan yang baik
% (best-practice hygiene), bukan fix utama.
Tf_deriv = 5;       % detik -- time constant filter derivative (first-order low-pass)

% --- REVISI DEADBAND: integral-only freeze (BUKAN full-output-freeze) --
% Deadband versi awal (single scalar simetris, membekukan SELURUH output
% u = u_prev saat |e|<=deadband) MEMPERBAIKI oscillation-nya sendiri
% dari 17.8 degC p2p menjadi 2.5 degC p2p -- tapi TIDAK menghilangkannya.
% Riset lanjutan (Control Global, "Deadband and Resolution Limit Cycle
% Causes and Fixes", controlglobal.com) menjelaskan ini adalah fenomena
% terkenal: membekukan SELURUH output (termasuk P & D) di sekitar
% setpoint secara struktural menyebabkan limit cycle terlepas dari
% tuning apa pun, karena proses (PV) tetap drift perlahan di dalam zona
% (P & D-nya "diam" walau PV bergerak), lalu melompat keluar zona,
% terkoreksi kembali, berulang. Solusi (ESPHome PID Climate,
% esphome.io/components/climate/pid + Control Global): deadband HANYA
% membekukan akumulasi INTEGRAL (sum_e) -- P dan D tetap dihitung &
% berkontribusi ke output seperti biasa selama di dalam zona, sehingga
% PV yang drift tetap direspons secara proporsional/derivatif, tidak
% dibiarkan bebas.
%
% Parameter berikut mendukung 3 lapis mitigasi tambahan (opsional,
% masing-masing bisa dinonaktifkan): threshold asimetris (dead_hi !=
% dead_lo), gain tapering (taper<1) di dalam zona, dan output averaging
% (n_avg>1) di dalam zona.
%
% HASIL PENGUJIAN BERTAHAP (sesuai metodologi yang diminta -- setiap
% layer diuji satu per satu, diukur osilasi p2p pada 600s terakhir dari
% simulasi 1800s, setpoint 150 degC):
%   Baseline lama (full-output-freeze, deadband=1.0 simetris) : 2.517 degC p2p
%   Layer 1 saja  (integral-only freeze, dead_hi=dead_lo=1.0) : 0.000  degC p2p  <-- SUDAH SEMPURNA
%   + Layer 2     (threshold asimetris: dead_hi=1.5,dead_lo=0.75) : 0.000 degC p2p (tetap sempurna, tidak merusak)
%   + Layer 3     (gain tapering taper=0.4)                   : 14.143 degC p2p  <-- JUSTRU MEMBURUK DRASTIS
%   + Layer 4     (output averaging n_avg=10, di atas Layer 3) : 10.409 degC p2p  <-- masih jauh lebih buruk dari Layer 1+2
%
% Kesimpulan: Layer 1 (integral-only freeze) SAJA sudah menghilangkan
% limit cycle sepenuhnya (diverifikasi juga pada setpoint 170 & 180 --
% p2p = 0.0000 degC di ketiganya). Layer 2 (asimetris) dipertahankan
% aktif karena tidak merusak dan merupakan praktik baik (Tact tidak
% dibiarkan overheat lama di atas Tref). Layer 3 & 4 DINONAKTIFKAN
% (taper=1, n_avg=1) karena TERBUKTI kontraproduktif untuk kombinasi
% plant+controller ini -- mengurangi otoritas Kp/Kd di dalam deadband
% (Layer 3) membuat sistem kurang responsif terhadap drift PV di dalam
% zona, justru MEMUNCULKAN KEMBALI osilasi (bukan noise, tapi limit
% cycle baru dari otoritas koreksi yang terlalu lemah). Mekanisme Layer
% 3 & 4 TETAP ada di kode (bukan dihapus) supaya bisa diuji ulang kalau
% parameter plant/PID berubah signifikan setelah kalibrasi dengan data
% fisik nanti -- lihat pidCode di bagian 7.
dead_hi = 1.5;   % degC -- batas atas error (Tact boleh sampai 1.5 degC di bawah Tref) sebelum keluar zona deadband
dead_lo = 0.75;  % degC -- batas bawah error (Tact boleh sampai 0.75 degC di atas Tref) sebelum keluar zona -- dibuat lebih ketat dari dead_hi supaya tidak overheat lama
taper   = 1.0;   % 0-1 -- pengali Kp/Ki/Kd di DALAM zona deadband. 1.0 = NONAKTIF (lihat hasil pengujian: taper<1 justru memburuk)
n_avg   = 1;     % jumlah sample output yang dirata-rata SAAT di dalam zona deadband. 1 = NONAKTIF (tanpa averaging)

%% ---------------------------------------------------------------
%% 3. PARAMETER AKTUATOR (servo + katup gas)
%% ---------------------------------------------------------------
out_min = 0;      % % bukaan katup minimum
out_max = 100;    % % bukaan katup maksimum
servo_tau = 0.7;  % detik -- lag mekanis servo (asumsi, 0.5-1s)

%% ---------------------------------------------------------------
%% 4. PARAMETER SIMULASI
%% ---------------------------------------------------------------
dt = 1;            % step size solver (detik). dt pada hukum adaptasi
                   % (delta_e/dt, sum_e += e*dt) mengikuti fixed step
                   % size solver ini secara langsung.
stop_time = 1800;  % detik per skenario setpoint
Tref_value = 150;  % default setpoint (di-override per run oleh script simulasi)

%% Simpan semua parameter ke base workspace (dipakai oleh model)
varNames = {'tau','L','K','T_ambient','Kp0','Ki0','Kd0','lambda_p', ...
    'lambda_i','lambda_d','Kp_max','Ki_max','Kd_max','Tf_deriv','dead_hi', ...
    'dead_lo','taper','n_avg','out_min','out_max','servo_tau','dt', ...
    'stop_time','Tref_value'};
for i = 1:numel(varNames)
    assignin('base', varNames{i}, eval(varNames{i}));
end

%% ---------------------------------------------------------------
%% 5. BANGUN MODEL SIMULINK SECARA TERPROGRAM
%% ---------------------------------------------------------------
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'ode1', 'FixedStep', 'dt', 'StopTime', 'stop_time');

% --- Setpoint (Step, konstan sejak t=0) ---
add_block('simulink/Sources/Step', [modelName '/Setpoint']);
set_param([modelName '/Setpoint'], 'Position', [30 88 60 112]);
set_param([modelName '/Setpoint'], 'Time', '0', 'Before', 'Tref_value', 'After', 'Tref_value');

% --- Adaptive PID controller sebagai MATLAB Function block ---
% Block ini punya persistent variable (e_prev, sum_e) untuk state
% adaptif -- Simulink MELARANG persistent variable pada block dengan
% sample time kontinu/inherited dari loop kontinu (error: "invalid when
% the block specifies or inherits a continuous sample time"). Karena
% plant (Transfer Fcn + Transport Delay) bersifat kontinu, MATLAB
% Function block ini perlu dipaksa berjalan pada laju DISKRIT (dt).
% Solusi robust: pasang Zero-Order Hold (ZOH) ber-SampleTime=dt pada
% kedua sinyal input (Tref dan Tact) SEBELUM masuk ke block ini (lihat
% bagian 5b). Ini sekaligus MEREPRESENTASIKAN kondisi fisik yang
% sebenarnya: ESP32 men-sampling suhu & meng-update kontrol pada
% interval waktu diskrit (bukan kontinu sempurna), sehingga ZOH di sini
% bukan sekadar trik pemrograman melainkan representasi loop kontrol
% digital yang realistis.
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/AdaptivePID']);
set_param([modelName '/AdaptivePID'], 'Position', [220 40 410 160]);

% --- ZOH: paksa Tref & Tact tersampling diskrit sebelum masuk controller ---
add_block('simulink/Discrete/Zero-Order Hold', [modelName '/ZOH_Tref']);
set_param([modelName '/ZOH_Tref'], 'Position', [150 60 180 80]);
set_param([modelName '/ZOH_Tref'], 'SampleTime', 'dt');

add_block('simulink/Discrete/Zero-Order Hold', [modelName '/ZOH_Tact']);
set_param([modelName '/ZOH_Tact'], 'Position', [150 120 180 140]);
set_param([modelName '/ZOH_Tact'], 'SampleTime', 'dt');

% --- Konstanta parameter controller & aktuator (input ke MATLAB Fcn) ---
paramNames = {'Kp0','Ki0','Kd0','lambda_p','lambda_i','lambda_d', ...
    'Kp_max','Ki_max','Kd_max','out_min','out_max','dt','Tf_deriv', ...
    'dead_hi','dead_lo','taper','n_avg'};
yPos = 200;
for i = 1:numel(paramNames)
    blkName = [modelName '/' paramNames{i} '_c'];
    add_block('simulink/Sources/Constant', blkName);
    w = 40 + numel(paramNames{i})*4;
    set_param(blkName, 'Position', [30 yPos 30+w yPos+20]);
    set_param(blkName, 'Value', paramNames{i});
    yPos = yPos + 35;
end

% --- Saturation block (aktuator limit 0-100%) ---
add_block('simulink/Discontinuities/Saturation', [modelName '/ActuatorSat']);
set_param([modelName '/ActuatorSat'], 'Position', [400 90 430 120]);
set_param([modelName '/ActuatorSat'], 'UpperLimit', 'out_max', 'LowerLimit', 'out_min');

% --- Servo lag (first-order, time constant servo_tau) ---
add_block('simulink/Continuous/Transfer Fcn', [modelName '/ServoLag']);
set_param([modelName '/ServoLag'], 'Position', [470 90 520 120]);
set_param([modelName '/ServoLag'], 'Numerator', '[1]', 'Denominator', '[servo_tau 1]');

% --- Plant: Transport Delay (dead time L) then Transfer Fcn (first order) ---
add_block('simulink/Continuous/Transport Delay', [modelName '/DeadTime']);
set_param([modelName '/DeadTime'], 'Position', [560 90 600 120]);
set_param([modelName '/DeadTime'], 'DelayTime', 'L');

add_block('simulink/Continuous/Transfer Fcn', [modelName '/PlantFOPDT']);
set_param([modelName '/PlantFOPDT'], 'Position', [640 90 690 120]);
set_param([modelName '/PlantFOPDT'], 'Numerator', '[K]', 'Denominator', '[tau 1]');

% --- Tact = T_ambient + kenaikan suhu plant ---
add_block('simulink/Sources/Constant', [modelName '/AmbientTemp']);
set_param([modelName '/AmbientTemp'], 'Position', [640 160 670 190]);
set_param([modelName '/AmbientTemp'], 'Value', 'T_ambient');

add_block('simulink/Math Operations/Sum', [modelName '/TempSum']);
set_param([modelName '/TempSum'], 'Position', [730 110 760 150]);
set_param([modelName '/TempSum'], 'Inputs', '++');

% --- Scope & logging ---
add_block('simulink/Sinks/Scope', [modelName '/TempScope']);
set_param([modelName '/TempScope'], 'Position', [820 60 850 90]);

add_block('simulink/Sinks/To Workspace', [modelName '/Tact_log']);
set_param([modelName '/Tact_log'], 'Position', [820 120 900 150]);
set_param([modelName '/Tact_log'], 'VariableName', 'Tact_out', 'SaveFormat', 'Structure With Time');

add_block('simulink/Sinks/To Workspace', [modelName '/Tref_log']);
set_param([modelName '/Tref_log'], 'Position', [150 220 230 250]);
set_param([modelName '/Tref_log'], 'VariableName', 'Tref_out', 'SaveFormat', 'Structure With Time');

add_block('simulink/Sinks/To Workspace', [modelName '/u_log']);
set_param([modelName '/u_log'], 'Position', [400 160 480 190]);
set_param([modelName '/u_log'], 'VariableName', 'u_out', 'SaveFormat', 'Structure With Time');

%% ---------------------------------------------------------------
%% 6. HUBUNGKAN BLOK-BLOK NON-MATLABFCN TERLEBIH DAHULU
%% ---------------------------------------------------------------
add_line(modelName, 'ActuatorSat/1', 'ServoLag/1', 'autorouting', 'on');
add_line(modelName, 'ServoLag/1', 'DeadTime/1', 'autorouting', 'on');
add_line(modelName, 'DeadTime/1', 'PlantFOPDT/1', 'autorouting', 'on');
add_line(modelName, 'PlantFOPDT/1', 'TempSum/1', 'autorouting', 'on');
add_line(modelName, 'AmbientTemp/1', 'TempSum/2', 'autorouting', 'on');
add_line(modelName, 'TempSum/1', 'TempScope/1', 'autorouting', 'on');
add_line(modelName, 'TempSum/1', 'Tact_log/1', 'autorouting', 'on');
add_line(modelName, 'Setpoint/1', 'Tref_log/1', 'autorouting', 'on');
add_line(modelName, 'AdaptivePID/1', 'u_log/1', 'autorouting', 'on');
add_line(modelName, 'AdaptivePID/1', 'ActuatorSat/1', 'autorouting', 'on');

%% ---------------------------------------------------------------
%% 7. ISI SCRIPT MATLAB FUNCTION BLOCK (Adaptive PID + anti-windup)
%% ---------------------------------------------------------------
% ANTI-WINDUP: integrator (sum_e) hanya diakumulasi jika output
% sebelum-saturasi (u_unsat, dihitung dengan kandidat sum_e baru)
% TIDAK melebihi batas saturasi aktuator [out_min, out_max]. Ini WAJIB
% karena tanpa anti-windup, saat pemanasan awal dari 30 degC ke setpoint
% (misal 150 degC), error besar bertahan lama sementara plant (time
% constant besar + dead time) belum sempat merespons. Tanpa anti-windup,
% sum_e akan terus membesar ("integrator windup") selama u sudah
% saturasi di 100%, sehingga saat suhu akhirnya mendekati setpoint,
% kontribusi Ki*sum_e masih sangat besar dan menyebabkan overshoot
% parah. Dengan clamping ini, integrator berhenti "menggunung" begitu
% aktuator saturasi.
% --- FIX #1: derivative filter + error deadband (best-practice hygiene,
% lihat catatan riset di bagian 2 parameter controller) -----------------
% Referensi: derivative low-pass filter ("Improving the Beginner's PID -
% Derivative Kick", brettbeauregard.com/blog/2011/04/improving-the-
% beginners-pid-derivative-kick); error deadband (ESPHome PID Climate
% docs esphome.io/components/climate/pid; Powerblanket PID Temperature
% Controller Guide).
%
% --- FIX #2 (AKAR MASALAH SEBENARNYA): Ki dijadwalkan ulang berdasarkan
% |e(k)| (error SEKARANG), BUKAN |sum_e| (akumulasi integral) ----------
% Setelah FIX #1 diterapkan, osilasi TIDAK banyak berkurang (p2p tetap
% ~17-18 degC) -- membuktikan derivative BUKAN penyebab utama. Uji isolasi
% (menonaktifkan lambda_i, lambda_d, lambda_p satu per satu secara
% terpisah dalam simulasi) menunjukkan HANYA menonaktifkan lambda_i yang
% menurunkan osilasi drastis dari 17.8 degC menjadi 2.5 degC p2p --
% menonaktifkan lambda_p atau lambda_d praktis tidak berpengaruh.
% Kesimpulan: hukum "Ki(k) = Ki0 + lambda_i*|sum_e|" (spesifikasi awal)
% menciptakan UMPAN BALIK POSITIF -- sum_e adalah akumulasi (satuan
% degC*detik) yang secara alami jauh lebih besar & terus tumbuh
% dibanding error sesaat, sehingga begitu sum_e membesar, GAIN Ki ikut
% membesar dan MENGALIKAN sum_e yang sudah besar itu -- kontribusi
% integral tumbuh super-linear, menyebabkan overshoot besar lalu ringing
% berkelanjutan saat sum_e berayun melewati nol di sisi lain. ISI Kp
% dijadwalkan berdasar |e(k)| (bukan basis internal integrator) TERBUKTI
% aman (menonaktifkannya tidak mengubah osilasi) -- pola scheduling yang
% sama diterapkan di sini untuk Ki, konsisten dengan gain-scheduling
% standar (skedul berdasar variabel yang diukur/independen, bukan
% berdasar state internal controller sendiri) sekaligus tetap
% mempertahankan semangat "Adaptive PID" (Kp, Ki, Kd semua tetap
% responsif terhadap besar error).
% --- FIX #3 (REVISI DEADBAND): integral-only freeze, threshold asimetris,
% gain tapering, output averaging -- lihat catatan riset lengkap di
% bagian 2 parameter controller (dead_hi/dead_lo/taper/n_avg). Deadband
% versi sebelumnya (full-output-freeze, u=u_prev) mengurangi osilasi dari
% 17.8 ke 2.5 degC p2p tapi TIDAK menghilangkannya -- pola limit cycle
% terkenal dari membekukan SELURUH output (referensi: Control Global,
% "Deadband and Resolution Limit Cycle Causes and Fixes"). Di sini HANYA
% integrator (sum_e) yang dibekukan di dalam deadband -- P dan D tetap
% aktif -- menggunakan pola yang SAMA dengan anti-windup saturasi yang
% sudah ada (kondisi "jangan akumulasi" digabung: dalam deadband ATAU
% saturasi aktuator).
% --- FIX #4: Kd dijadwalkan dari d_filt (bukan delta_e mentah) --------
% Lihat catatan lengkap di bagian 2 (definisi Kd0/lambda_d/Kp_max dst).
%
% --- FIX #5: gain limits (Kp_max/Ki_max/Kd_max) -----------------------
% Jaring pengaman -- lihat catatan di bagian 2.
%
% --- FIX #6: anti-windup conditional integration ----------------------
% Versi sebelumnya (clamp total): begitu u_candidate melewati batas
% saturasi, integrator SELALU dibekukan, tidak peduli arah error. Ini
% valid untuk KASUS UMUM, tapi kurang tepat untuk sistem heating-only
% (u_min=0, u_max=100, TIDAK ADA pendinginan aktif): kalau error sudah
% berbalik arah (mis. u_candidate masih >100% dari sisa integral lama,
% padahal Tact sudah lewat Tref sehingga e<0), integrator SEHARUSNYA
% tetap boleh berkurang (unwind) supaya sistem cepat keluar dari kondisi
% saturasi semu tsb, bukan dibekukan total. Conditional integration:
% akumulasi TETAP diizinkan selama arah perubahannya (tanda e) membantu
% MENGURANGI saturasi, hanya diblokir kalau arahnya akan MEMPERPARAH
% saturasi yang sudah terjadi. Deadband (Layer 1, TERBUKTI lewat
% pengujian menghilangkan limit cycle sepenuhnya) tetap memblokir total
% seperti sebelumnya -- tidak diubah.
MAXN = 15; % ukuran buffer tetap untuk output averaging (layer 4)
pidCode = [ ...
    "function u = adaptive_pid(Tref, Tact, Kp0, Ki0, Kd0, lambda_p, lambda_i, lambda_d, Kp_max, Ki_max, Kd_max, out_min, out_max, dt, Tf, dead_hi, dead_lo, taper, n_avg)"; ...
    "%#codegen"; ...
    sprintf("persistent e_prev sum_e d_filt initialized u_buf u_buf_idx buf_count"); ...
    sprintf("MAXN = %d;", MAXN); ...
    "if isempty(initialized)"; ...
    "    e_prev = 0;"; ...
    "    sum_e = 0;"; ...
    "    d_filt = 0;"; ...
    "    u_buf = zeros(1, MAXN);"; ...
    "    u_buf_idx = 0;"; ...
    "    buf_count = 0;"; ...
    "    initialized = true;"; ...
    "end"; ...
    ""; ...
    "e = Tref - Tact;"; ...
    "delta_e = e - e_prev;"; ...
    ""; ...
    "% -- derivative low-pass filter (first-order, time constant Tf) --"; ...
    "alpha = dt / (Tf + dt);"; ...
    "d_filt = d_filt + alpha * (delta_e/dt - d_filt);"; ...
    ""; ...
    "Kp = min(Kp0 + lambda_p * abs(e), Kp_max);"; ...
    "Kd = min(Kd0 + lambda_d * abs(d_filt), Kd_max); % dijadwalkan dari d_filt (FIX #4), bukan delta_e mentah"; ...
    "Ki = min(Ki0 + lambda_i * abs(e), Ki_max); % dijadwalkan dari |e|, BUKAN |sum_e| -- lihat catatan FIX #2"; ...
    ""; ...
    "% -- threshold asimetris: dead_hi (Tact di bawah Tref) vs dead_lo (Tact di atas Tref) --"; ...
    "inDeadband = (e <= dead_hi) && (e >= -dead_lo);"; ...
    ""; ...
    "% -- gain tapering di dalam deadband (P & D tetap aktif, tidak dibekukan) --"; ...
    "if inDeadband"; ...
    "    Kp_eff = Kp * taper;"; ...
    "    Ki_eff = Ki * taper;"; ...
    "    Kd_eff = Kd * taper;"; ...
    "else"; ...
    "    Kp_eff = Kp;"; ...
    "    Ki_eff = Ki;"; ...
    "    Kd_eff = Kd;"; ...
    "end"; ...
    ""; ...
    "% -- anti-windup: conditional integration (FIX #6) + deadband integral-only freeze --"; ...
    "sum_e_candidate = sum_e + e * dt;"; ...
    "u_candidate = Kp_eff*e + Ki_eff*sum_e_candidate + Kd_eff*d_filt;"; ...
    "if inDeadband"; ...
    "    doAccumulate = false; % Layer 1 -- TERBUKTI menghilangkan limit cycle, tidak diubah"; ...
    "elseif u_candidate > out_max"; ...
    "    doAccumulate = (e < 0); % izinkan integral MENGECIL (unwind) kalau error sudah berbalik"; ...
    "elseif u_candidate < out_min"; ...
    "    doAccumulate = (e > 0); % izinkan integral MEMBESAR kembali kalau error sudah berbalik"; ...
    "else"; ...
    "    doAccumulate = true;"; ...
    "end"; ...
    ""; ...
    "if doAccumulate"; ...
    "    sum_e = sum_e_candidate;"; ...
    "    u_raw = u_candidate;"; ...
    "else"; ...
    "    u_raw = Kp_eff*e + Ki_eff*sum_e + Kd_eff*d_filt;"; ...
    "end"; ...
    "u_raw = min(max(u_raw, out_min), out_max);"; ...
    ""; ...
    "% -- output averaging di dalam deadband (buffer sirkular ukuran MAXN) --"; ...
    "% FIX #7: hitung sample VALID (buf_count) supaya slot buffer yang"; ...
    "% belum pernah terisi (masih nol dari inisialisasi) tidak ikut"; ...
    "% dirata-rata dan menurunkan output secara tidak wajar."; ...
    "u_buf_idx = mod(u_buf_idx, MAXN) + 1;"; ...
    "u_buf(u_buf_idx) = u_raw;"; ...
    "buf_count = min(buf_count + 1, MAXN);"; ...
    "if inDeadband && n_avg > 1"; ...
    "    nn = min([round(n_avg), MAXN, buf_count]);"; ...
    "    s = 0;"; ...
    "    for k = 0:(nn-1)"; ...
    "        pos = mod(u_buf_idx - 1 - k, MAXN) + 1;"; ...
    "        s = s + u_buf(pos);"; ...
    "    end"; ...
    "    u = s / nn;"; ...
    "else"; ...
    "    u = u_raw;"; ...
    "end"; ...
    ""; ...
    "e_prev = e;"; ...
    "end" ...
    ];
pidScript = strjoin(cellstr(pidCode), newline);

% Set kode function di dalam MATLAB Function block via Stateflow API
sfObj = sfroot;
chartObj = sfObj.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/AdaptivePID']);
chartObj.Script = pidScript;
% Catatan: SampleTime block ini dibiarkan 'inherited' (-1) -- laju
% diskritnya diperoleh dari sinyal input yang sudah di-ZOH (lihat
% ZOH_Tref/ZOH_Tact di bagian 5), bukan diset langsung pada objek chart
% (properti tsb terbukti tidak reliable diset via scripting API).

% Update diagram supaya port-port MATLAB Function ter-generate sesuai
% signature (Tref, Tact, Kp0, Ki0, Kd0, lambda_p, lambda_i, lambda_d,
% Kp_max, Ki_max, Kd_max, out_min, out_max, dt, Tf_deriv, dead_hi,
% dead_lo, taper, n_avg) -> port 1..19, output port 1 = u
set_param(modelName, 'SimulationCommand', 'update');

%% ---------------------------------------------------------------
%% 8. SAMBUNGKAN INPUT-INPUT MATLAB FUNCTION BLOCK
%% ---------------------------------------------------------------
% Port 1 = Tref, Port 2 = Tact (feedback), Port 3..11 = parameter constants
% Tref & Tact melewati ZOH (sample-and-hold, laju dt) dahulu supaya
% MATLAB Function block berjalan pada laju diskrit yang valid.
add_line(modelName, 'Setpoint/1', 'ZOH_Tref/1', 'autorouting', 'on');
add_line(modelName, 'ZOH_Tref/1', 'AdaptivePID/1', 'autorouting', 'on');
add_line(modelName, 'TempSum/1', 'ZOH_Tact/1', 'autorouting', 'on');
add_line(modelName, 'ZOH_Tact/1', 'AdaptivePID/2', 'autorouting', 'on');
for i = 1:numel(paramNames)
    srcPort = [paramNames{i} '_c/1'];
    dstPort = sprintf('AdaptivePID/%d', i + 2);
    add_line(modelName, srcPort, dstPort, 'autorouting', 'on');
end

%% ---------------------------------------------------------------
%% 8b. INIT FCN CALLBACK -- supaya model BISA DIBUKA & DIJALANKAN
%%     LANGSUNG dari Simulink (double-click .slx) TANPA harus
%%     menjalankan skrip .m ini/run_deep_fryer_sim.m dulu.
%% ---------------------------------------------------------------
% Tanpa ini, membuka .slx secara langsung akan error "Unrecognized
% function or variable 'stop_time'" (atau tau/Kp0/dst) karena
% variabel-variabel tersebut hanya ada di base workspace selama sesi
% MATLAB yang menjalankan build_deep_fryer_model()/run_deep_fryer_sim.m
% -- begitu sesi MATLAB ditutup dan .slx dibuka ulang, base workspace
% kosong lagi. InitFcn berjalan otomatis setiap kali model di-load,
% sehingga variabel ini SELALU tersedia. Dibuat otomatis dari nilai
% varNames yang sama di atas, supaya tidak ada duplikasi nilai yang bisa
% membuat drift antara skrip build dan model tersimpan.
%
% PENTING: setiap baris dibungkus "if ~exist(...,'var')" (guard) supaya
% InitFcn TIDAK menimpa variabel yang sengaja sudah di-assign ke base
% workspace SEBELUM sim() dipanggil -- InitFcn Simulink berjalan ulang
% di SETIAP awal simulasi (bukan cuma sekali saat model dibuka), jadi
% tanpa guard ini, Tref_value yang di-set berbeda-beda oleh loop 3
% setpoint di run_deep_fryer_sim.m akan selalu ditimpa balik ke nilai
% default (150) setiap kali sim() dipanggil, membuat setpoint 170/180
% diam-diam tersimulasi seolah masih 150.
initFcnLines = cell(1, numel(varNames));
for i = 1:numel(varNames)
    initFcnLines{i} = sprintf('if ~exist(''%s'',''var''); %s = %.10g; end', ...
        varNames{i}, varNames{i}, eval(varNames{i}));
end
set_param(modelName, 'InitFcn', strjoin(initFcnLines, newline));

%% ---------------------------------------------------------------
%% 9. SIMPAN MODEL
%% ---------------------------------------------------------------
save_system(modelName, fullfile(pwd, [modelName '.slx']));
fprintf('Model %s.slx berhasil dibangun di: %s\n', modelName, fullfile(pwd, [modelName '.slx']));

close_system(modelName, 1);

end
