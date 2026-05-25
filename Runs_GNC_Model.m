if ~exist('A','var'),         init_GNC;        end
if ~bdIsLoaded('GNC_System'), build_GNC_model; end
 
simOut = sim('GNC_System');
 
vars = {'delta_c','FPA_cmd','FPA_act','range_m','alt_m','miss_m'};
for k = 1:numel(vars)
    if ~exist(vars{k},'var')
        try
            v = simOut.(vars{k});
            if isnumeric(v),  assignin('base', vars{k}, v); end
        catch
            try
                v = simOut.get(vars{k});
                if isobject(v), v = v.Data; end
                assignin('base', vars{k}, v);
            catch
            end
        end
    end
end
 
if ~exist('range_m','var') || isempty(range_m)
    disp('No data — check model is built and wired correctly.');
    return
end
 
t  = (0:length(range_m)-1)' * 0.005;
n2 = min(length(FPA_cmd), length(FPA_act));
 
figure('Name','GNC Results','NumberTitle','off', ...
       'Units','normalized','Position',[0.03 0.03 0.94 0.92]);
 
subplot(3,2,1);
plot(t, alt_m/1000,'b','LineWidth',1.5); grid on; hold on;
yline(ELEV_TARGET/1000,'r--'); yline(ELEV_INIT/1000,'g--');
xlabel('t (s)'); ylabel('km'); title('Altitude');
legend('actual','target','launch','Location','best');
 
subplot(3,2,2);
plot(t, range_m/1000,'k','LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('km'); title('Range to target');
 
subplot(3,2,3);
plot(t(1:n2), FPA_cmd(1:n2)*180/pi,'r--','LineWidth',1.5); hold on;
plot(t(1:n2), FPA_act(1:n2)*180/pi,'b',  'LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('deg'); title('FPA');
legend('cmd','actual','Location','best');
 
subplot(3,2,4);
plot(t, delta_c*180/pi,'m','LineWidth',1.5); grid on; hold on;
yline(25,'r--'); yline(-25,'r--');
xlabel('t (s)'); ylabel('deg'); title('Canard deflection');
 
subplot(3,2,5);
plot(t, miss_m/1000,'Color',[0 0.55 0],'LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('km'); title('Miss distance');
 
subplot(3,2,6);
plot(range_m/1000, alt_m/1000,'b','LineWidth',2); grid on; hold on;
scatter(r_init/1000, ELEV_INIT/1000,   80,'g','filled');
scatter(0,           ELEV_TARGET/1000, 80,'r','filled');
xlabel('range (km)'); ylabel('alt (km)'); title('Trajectory');
legend('path','launch','target','Location','best');
 
sgtitle('GNC System — Simulation Results');
 
fprintf('\nTime to target  : %.2f s\n',  t(end));
fprintf('Final range     : %.1f m\n',   range_m(end));
fprintf('Miss distance   : %.1f m\n',   miss_m(end));
fprintf('Peak delta_c    : %.2f deg\n', max(abs(delta_c))*180/pi);
fprintf('FPA error (rms) : %.3f deg\n', rms((FPA_cmd(1:n2)-FPA_act(1:n2))*180/pi));
 