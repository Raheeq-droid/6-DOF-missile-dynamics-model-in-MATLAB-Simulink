if ~exist('A','var'),         init_GNC;        end
if ~bdIsLoaded('GNC_System'), build_GNC_model; end

FG_IP   = '127.0.0.1';
FG_PORT = 5500;
FG_DT   = 0.05;

sock = udpport('datagram','IPV4','LocalPort',0,'Timeout',2);

fprintf('Streaming to FlightGear %s:%d\n', FG_IP, FG_PORT);
fprintf('Make sure FlightGear is already open.\n\n');

t_sim = 0;

while t_sim < 150
    t_end = min(t_sim + FG_DT, 150);

    try
        simOut = sim('GNC_System', 'StartTime',num2str(t_sim), ...
                                   'StopTime', num2str(t_end));
    catch e
        fprintf('t=%.2f: %s\n', t_sim, e.message);
        break;
    end

    vars = {'range_m','alt_m','FPA_act','delta_c','FPA_cmd','miss_m'};
    for k = 1:numel(vars)
        if ~exist(vars{k},'var')
            try
                v = simOut.(vars{k});
                if isnumeric(v), assignin('base',vars{k},v); end
            catch, end
        end
    end

    if exist('range_m','var') && ~isempty(range_m)
        rng = range_m(end);
        alt = alt_m(end) * m2f;
        fpa = FPA_act(end);
    else
        rng = r_init;  alt = ELEV_INIT * m2f;  fpa = FPA_INIT;
    end

    dt_r = r_init - rng;
    lat  = LAT_INIT + (dt_r * cos(yaw_init) / R_earth) * (180/pi);
    lon  = LON_INIT + (dt_r * sin(yaw_init) / (R_earth * cosd(LAT_INIT))) * (180/pi);

    try
        pkt = build_fgnetfdm(lat, lon, alt, 0, fpa, yaw_init);
        write(sock, pkt, 'uint8', FG_IP, FG_PORT);
    catch, end

    t_sim = t_end;

    if mod(round(t_sim / FG_DT), 20) == 0
        fprintf('t=%5.1f s  range=%6.0f m  alt=%6.0f m\n', t_sim, rng, alt/m2f);
    end

    if exist('range_m','var') && rng < 100 && t_sim > 10
        fprintf('Hit at t=%.2f s\n', t_sim);
        break;
    end
end

clear sock;

% Results plots
if ~exist('range_m','var') || isempty(range_m), return; end

t  = (0:length(range_m)-1)' * FG_DT;
n2 = min(length(FPA_cmd), length(FPA_act));

figure('Name','GNC 3D Results','NumberTitle','off', ...
       'Units','normalized','Position',[0.03 0.03 0.94 0.92]);

subplot(2,2,1);
plot(t, alt_m/1000,'b','LineWidth',1.5); grid on; hold on;
yline(ELEV_TARGET/1000,'r--'); yline(ELEV_INIT/1000,'g--');
xlabel('t (s)'); ylabel('km'); title('Altitude');
legend('actual','target','launch','Location','best');

subplot(2,2,2);
plot(t, range_m/1000,'k','LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('km'); title('Range to target');

subplot(2,2,3);
plot(t(1:n2), FPA_act(1:n2)*180/pi,'b','LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('deg'); title('Flight-path angle');

subplot(2,2,4);
plot(range_m/1000, alt_m/1000,'b','LineWidth',2); grid on; hold on;
scatter(r_init/1000, ELEV_INIT/1000,   80,'g','filled');
scatter(0,           ELEV_TARGET/1000, 80,'r','filled');
xlabel('range (km)'); ylabel('alt (km)'); title('Trajectory');
legend('path','launch','target','Location','best');

sgtitle('GNC System — 3D Simulation Results');