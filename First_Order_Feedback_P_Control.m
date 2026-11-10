% First_Order_Feedback_P_Control.m
% آزمایش: تحلیل سیستم مرتبه اول با فیدبک منفی واحد و کنترل‌کننده تناسبی
% نیازمند MATLAB و Control System Toolbox

clear; clc; close all;

%% 1) پارامترهای قابل تنظیم
tau = 3.3e-3;                       % ثابت زمانی فرآیند (ثانیه)
Kp_values = [0.1 0.5 1 2 5 10];     % بهره‌های تناسبی مورد بررسی
r0 = 1;                             % دامنه ورودی پله
t_end = max(8*tau, 20e-3);          % زمان شبیه‌سازی
t = linspace(0, t_end, 3000);       % بردار زمان

if ~isscalar(tau) || tau <= 0
    error('پارامتر tau باید یک عدد مثبت باشد.');
end
if isempty(Kp_values) || any(Kp_values <= 0)
    error('تمام مقادیر Kp باید مثبت باشند.');
end

%% 2) تعریف مدل فرآیند، کنترل‌کننده و حلقه باز
s = tf('s');
G = 1/(tau*s + 1);                  % فرآیند مرتبه اول
fprintf('\n========== مدل ریاضی سیستم ==========\n');
fprintf('تابع تبدیل فرآیند G(s):\n'); G
fprintf('کنترل‌کننده تناسبی C(s)=Kp است.\n');
fprintf('تابع تبدیل حلقه باز L(s)=C(s)G(s) است.\n');

%% 3) محاسبه مدل‌ها و مشخصات برای هر بهره
n = numel(Kp_values);
T = cell(n,1);                      % تابع تبدیل حلقه بسته خروجی
L = cell(n,1);                      % تابع تبدیل حلقه باز
S = cell(n,1);                      % تابع حساسیت / خطای تعقیب
Uref = cell(n,1);                   % تابع تبدیل فرمان کنترل به مرجع

Kp = Kp_values(:);
DCGain = zeros(n,1);
SteadyStateError = zeros(n,1);
TauClosedLoop_ms = zeros(n,1);
Pole = zeros(n,1);
RiseTime_ms = zeros(n,1);
RiseTimeTheory_ms = zeros(n,1);
SettlingTime_ms = zeros(n,1);
SettlingTimeTheory_ms = zeros(n,1);
Overshoot_percent = zeros(n,1);
PeakValue = zeros(n,1);
PeakTime_ms = zeros(n,1);
Stable = strings(n,1);

for k = 1:n
    C = Kp(k);
    L{k} = minreal(C*G);
    T{k} = feedback(L{k},1);         % T(s)=L(s)/(1+L(s))
    S{k} = feedback(1,L{k});         % S(s)=1/(1+L(s))
    Uref{k} = minreal(C*S{k});       % U(s)/R(s)=C(s)/(1+L(s))

    DCGain(k) = dcgain(T{k});
    SteadyStateError(k) = abs(r0*(1-DCGain(k)));
    TauClosedLoop_ms(k) = 1e3*tau/(1+Kp(k));
    Pole(k) = pole(T{k});
    RiseTimeTheory_ms(k) = 1e3*log(9)*tau/(1+Kp(k)); % زمان خیز 10 تا 90 درصد
    SettlingTimeTheory_ms(k) = 1e3*(-log(0.02))*tau/(1+Kp(k));

    info = stepinfo(T{k}, 'SettlingTimeThreshold', 0.02);
    RiseTime_ms(k) = 1e3*info.RiseTime;
    SettlingTime_ms(k) = 1e3*info.SettlingTime;
    Overshoot_percent(k) = info.Overshoot;
    PeakValue(k) = info.Peak;
    PeakTime_ms(k) = 1e3*info.PeakTime;

    if all(real(pole(T{k})) < 0)
        Stable(k) = "پایدار";
    else
        Stable(k) = "ناپایدار";
    end
end

%% 4) نمایش جدول مشخصات زمانی و پایداری
Results = table(Kp, DCGain, SteadyStateError, TauClosedLoop_ms, ...
    RiseTimeTheory_ms, RiseTime_ms, SettlingTimeTheory_ms, ...
    SettlingTime_ms, Overshoot_percent, PeakValue, PeakTime_ms, Pole, Stable);
disp('========== جدول مشخصات سیستم حلقه‌بسته ==========');
disp(Results);

%% 5) نمایش توابع تبدیل حلقه باز، حلقه بسته، خطا و فرمان کنترل
for k = 1:n
    fprintf('\n---------- Kp = %g ----------\n', Kp(k));
    fprintf('L(s) = C(s)G(s):\n'); L{k}
    fprintf('T(s) = Y(s)/R(s):\n'); T{k}
    fprintf('S(s) = E(s)/R(s):\n'); S{k}
    fprintf('U(s)/R(s) = C(s)/(1+C(s)G(s)):\n'); Uref{k}
    fprintf('قطب حلقه‌بسته: %.6g rad/s\n', Pole(k));
    fprintf('خطای ماندگار پله واحد: %.6g\n', SteadyStateError(k));
end

%% 6) نمودار پاسخ پله فرآیند بدون کنترل
figure('Name','Open-loop plant response','Color','w');
step(r0*G,t); grid on;
title('پاسخ پله فرآیند بدون کنترل');
xlabel('زمان (s)'); ylabel('خروجی');

%% 7) نمودار پاسخ پله حلقه‌بسته برای بهره‌های مختلف
figure('Name','Closed-loop step responses','Color','w');
hold on; grid on;
for k = 1:n
    [y, tout] = step(r0*T{k},t);
    plot(tout, squeeze(y), 'LineWidth',1.5, ...
        'DisplayName',sprintf('K_p = %g',Kp(k)));
end
yline(r0,'--k','مرجع');
xlabel('زمان (s)'); ylabel('خروجی y(t)');
title('مقایسه پاسخ پله حلقه‌بسته برای بهره‌های مختلف');
legend('Location','southeast'); hold off;

%% 8) نمودار خطای تعقیب
figure('Name','Tracking error','Color','w');
hold on; grid on;
for k = 1:n
    [e, tout] = step(r0*S{k},t);
    plot(tout, squeeze(e), 'LineWidth',1.5, ...
        'DisplayName',sprintf('K_p = %g',Kp(k)));
end
xlabel('زمان (s)'); ylabel('خطا e(t)');
title('پاسخ خطای تعقیب برای بهره‌های مختلف');
legend('Location','northeast'); hold off;

%% 9) نمودار سیگنال کنترلی
figure('Name','Control effort','Color','w');
hold on; grid on;
for k = 1:n
    [u, tout] = step(r0*Uref{k},t);
    plot(tout, squeeze(u), 'LineWidth',1.5, ...
        'DisplayName',sprintf('K_p = %g',Kp(k)));
end
xlabel('زمان (s)'); ylabel('فرمان کنترل u(t)');
title('سیگنال کنترلی برای بهره‌های مختلف');
legend('Location','northeast'); hold off;

%% 10) نمودار بود حلقه‌بسته و حلقه باز برای بهره منتخب
selected_index = min(3,n);
figure('Name','Bode plots','Color','w');
bode(L{selected_index},T{selected_index}); grid on;
legend('حلقه باز L(s)','حلقه بسته T(s)','Location','best');
title(sprintf('نمودار بود برای K_p = %g',Kp(selected_index)));

%% 11) نقشه قطب‌ها و صفرهای حلقه‌بسته
figure('Name','Closed-loop pole-zero map','Color','w');
hold on; grid on;
for k = 1:n
    pzmap(T{k});
end
title('نقشه قطب‌ها و صفرهای سیستم حلقه‌بسته');
legend(arrayfun(@(x)sprintf('K_p = %g',x),Kp,'UniformOutput',false), ...
    'Location','best');
hold off;

%% 12) جمع‌بندی روابط تحلیلی
fprintf('\n========== روابط تحلیلی نهایی ==========\n');
fprintf('G(s) = 1/(tau*s+1)\n');
fprintf('C(s) = Kp\n');
fprintf('L(s) = Kp/(tau*s+1)\n');
fprintf('T(s) = Kp/(tau*s+1+Kp)\n');
fprintf('S(s) = (tau*s+1)/(tau*s+1+Kp)\n');
fprintf('U(s)/R(s) = Kp*(tau*s+1)/(tau*s+1+Kp)\n');
fprintf('بهره DC حلقه‌بسته = Kp/(1+Kp)\n');
fprintf('خطای ماندگار پله واحد = 1/(1+Kp)\n');
fprintf('ثابت زمانی حلقه‌بسته = tau/(1+Kp)\n');
fprintf('قطب حلقه‌بسته = -(1+Kp)/tau\n');
fprintf('زمان خیز 10-90%% = ln(9)*tau/(1+Kp)\n');
fprintf('زمان نشست 2%% = -ln(0.02)*tau/(1+Kp)\n');
fprintf('\nپایان تحلیل. تمام زمان‌ها در جدول بر حسب میلی‌ثانیه هستند.\n');
