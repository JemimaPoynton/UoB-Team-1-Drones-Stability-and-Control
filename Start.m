%% Inital Values
u0 = [ 0;
       0; % review elevator deflection limits
       0;
       0];

x0 = [22;
      0;
      0;
      0;
      0;
      0;
      0;
      0.05587;
      0];

Time = 400; % [s] Length of simulation

m = 150;

I = [107,   0, 0.6971; % see RCAM page 12 for format
     0,   93, 0;
     0.6971, 0, 196];
%%
u0 = [0;
    0;
    0;
    280
    0];

x0 = [21.9999986140590;
    0.532475749915017;
    3.52812483438128e-06;
    ];

%% Plot PID Response Data
figure()
plot(out.altitudeResponse.Time,out.altitudeResponse.Data(:,1)*(-1) + 1000,'blue-') % Corrected to altitude from Z down
hold on
plot(out.altitudeResponse.Time,out.altitudeResponse.Data(:,2)*(-1) + 1000,'black-', 'LineWidth',1)

xlim([40 150]);
ylabel('Altitude (m)')
xlabel('Time (s)')

legend('Demand', 'Response')
%%
plot(out.headingAngleResponse.Time,out.headingAngleResponse.Data(:,1),'blue-')
hold on
plot(out.headingAngleResponse.Time,out.headingAngleResponse.Data(:,2),'black-', 'LineWidth',1)

xlim([30 140]);
ylabel('Heading Angle (rad)')
xlabel('Time (s)')

legend('Demand', 'Response')
%%
figure()
plot(out.airspeedResponse.Time,out.airspeedResponse.Data(:,1),'blue-')
hold on
plot(out.airspeedResponse.Time,out.airspeedResponse.Data(:,2),'black-', 'LineWidth',1)

xlim([40 150]);
ylabel('Airspeed (m/s)')
xlabel('Time (s)')

legend('Demand', 'Response')

%%
figure()
plot(out.speedAltitude.Time,out.speedAltitude.Data(:,1),'black-')
hold on
plot(out.speedAltitude.Time,out.speedAltitude.Data(:,2),'blue-')
plot([13 13],[0 130], 'black:')

xlim([0 14]);
ylim([0 120])
xlabel('Time (s)')

legend('Airspeed (m/s)', 'Altitude (m)')

%% %% Plot PID Response Data VTOL
figure()
plot(out.VTOLZResponse.Time,out.VTOLZResponse.Data(:,1),'blue-') % Corrected to altitude from Z down
hold on
plot(out.VTOLZResponse.Time,out.VTOLZResponse.Data(:,2),'black-', 'LineWidth',1)

xlim([40 150]);
ylabel('Z Velocity (m/s)')
xlabel('Time (s)')

legend('Demand', 'Response')

figure()
plot(out.VTOLYResponse.Time,out.VTOLYResponse.Data(:,1),'blue-')
hold on
plot(out.VTOLYResponse.Time,out.VTOLYResponse.Data(:,2),'black-', 'LineWidth',1)

xlim([40 160]);
ylabel('Y Velocity (m/s)')
xlabel('Time (s)')

legend('Demand', 'Response')

figure()
plot(out.VTOLPitchResponse.Time,out.VTOLPitchResponse.Data(:,1),'blue-')
hold on
plot(out.VTOLPitchResponse.Time,out.VTOLPitchResponse.Data(:,2),'black-', 'LineWidth',1)

xlim([40 150]);
ylabel('Pitch (rad)')
xlabel('Time (s)')

legend('Demand', 'Response')

figure()
plot(out.VTOLXResponse.Time,out.VTOLXResponse.Data(:,1),'blue-')
hold on
plot(out.VTOLXResponse.Time,out.VTOLXResponse.Data(:,2),'black-', 'LineWidth',1)

xlim([40 160]);
ylabel('X Velocity (m/s)')
xlabel('Time (s)')


%%