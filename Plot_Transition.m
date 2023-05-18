%% Start VTOL Climb up for 16s
x0 = [0.0000001; % Very low forward speed for turbulence model
      -1;
      0;
      0;
      0;
      0;
      0;
      0;
      0];

m = 150;

I = [107,   0, 0.6971; % see RCAM page 12 for format
     0,   93, 0;
     0.6971, 0, 196];

transitionTime = 14;

VTOL = sim('VTOL_V4_Final_TRANSITION.slx');

%% Start Climb in Transition

x0 = [VTOL.TransitionStart.uvw.Data(end,1);
      VTOL.TransitionStart.uvw.Data(end,2);
      VTOL.TransitionStart.uvw.Data(end,3)
      VTOL.TransitionStart.pqr.Data(end,1);
      VTOL.TransitionStart.pqr.Data(end,2);
      VTOL.TransitionStart.pqr.Data(end,3)
      VTOL.TransitionStart.EA.Data(end,1);
      VTOL.TransitionStart.EA.Data(end,2);
      VTOL.TransitionStart.EA.Data(end,3)];

xyz0 = [VTOL.TransitionStart.Xe.Data(end,1);
        VTOL.TransitionStart.Xe.Data(end,2);
        VTOL.TransitionStart.Xe.Data(end,3)];

u05 = VTOL.thrust.Data(end,5);

%% Run Cruise Stage
Cruise = sim('Revised_Cruise_V4_MODIFIED_4.slx');

%% Setup landing 

x0 = [Cruise.landingConditions.uvw.Data(end,1);
      Cruise.landingConditions.uvw.Data(end,2);
      Cruise.landingConditions.uvw.Data(end,3)
      Cruise.landingConditions.pqr.Data(end,1);
      Cruise.landingConditions.pqr.Data(end,2);
      Cruise.landingConditions.pqr.Data(end,3)
      Cruise.landingConditions.EA.Data(end,1);
      Cruise.landingConditions.EA.Data(end,2);
      Cruise.landingConditions.EA.Data(end,3)];

xyz0 = [Cruise.landingConditions.Xe.Data(end,1);
        Cruise.landingConditions.Xe.Data(end,2);
        Cruise.landingConditions.Xe.Data(end,3)];

%% Run Landing Model
Landing = sim('VTOL_V4_Final_LANDING.slx');

%% Plot 3D path
figure()
plot3(Cruise.simout.Data(:,1),Cruise.simout.Data(:,2), Cruise.simout.Data(:,3)*(-1))
hold on

plot3(VTOL.VTOL.Data(:,1),VTOL.VTOL.Data(:,2), VTOL.VTOL.Data(:,3)*(-1))

plot3(Landing.VTOL.Data(:,1),Landing.VTOL.Data(:,2), Landing.VTOL.Data(:,3)*(-1))

ylim([-1900 1900])
xlim([0 24000])
zlim([0 530])

xlabel('X (North) [m]')
ylabel('Y (East) [m]')
zlabel('Altitude above SL [m]')

%% Plot VTOL Transition thrust
figure()
plot(VTOL.thrust.Time, VTOL.thrust.Data(:,1),'blue-', 'LineWidth',1)
hold on
plot(VTOL.thrust.Time, VTOL.thrust.Data(:,2), 'blue--','LineWidth',1)
plot(VTOL.thrust.Time, VTOL.thrust.Data(:,3),'black-','LineWidth',1)
plot(VTOL.thrust.Time, VTOL.thrust.Data(:,4), 'black--','LineWidth',1)


plot(Cruise.control.Time(1:96) + VTOL.thrust.Time(end), Cruise.control.Data(1:96,5).*426/1500,'blue-','LineWidth',1)
hold on
plot(Cruise.control.Time(1:96) + VTOL.thrust.Time(end), Cruise.control.Data(1:96,5).*426/1500, 'blue-','LineWidth',1)
plot(Cruise.control.Time(1:96) + VTOL.thrust.Time(end), Cruise.control.Data(1:96,5).*269.5/1500, 'black-','LineWidth',1)
plot(Cruise.control.Time(1:96) + VTOL.thrust.Time(end), Cruise.control.Data(1:96,5).*269.5/1500, 'black-','LineWidth',1)


plot(VTOL.thrust.Time, VTOL.thrust.Data(:,4) + VTOL.thrust.Data(:,3) + VTOL.thrust.Data(:,2) + VTOL.thrust.Data(:,1), 'black:','LineWidth',1)
plot(VTOL.thrust.Time, VTOL.thrust.Data(:,4) + VTOL.thrust.Data(:,3) + VTOL.thrust.Data(:,2) + VTOL.thrust.Data(:,1), 'black:','LineWidth',1)
plot(Cruise.control.Time(1:96) + VTOL.thrust.Time(end), Cruise.control.Data(1:96,5).*(269.5+426)*2/1500,'black:','LineWidth',1)

legend('Front Left','Front Right','Rear Left','Rear Right','','','','','Total')
ylabel('Thrust (N)')
xlabel('Time (s)')

%% Plot rear thrust
figure() % This isn't overly interesting so maybe doesn't need to be included in the report
plot(VTOL.thrust.Time, VTOL.thrust.Data(:,5))
hold on
plot(Cruise.control.Time + VTOL.thrust.Time(end), Cruise.control.Data(:,4))
xlim([16 80])
ylim([0 310])

%% Performance of transition
figure()
plot(VTOL.TransitionStart.Ve.Time, VTOL.TransitionStart.Ve.Data(:,1),'black-')
hold on
plot(Cruise.airspeed.Time(end) + VTOL.thrust.Time(end) + Landing.TransitionStart.Ve.Time, Landing.TransitionStart.Ve.Data(:,1),'black-')
plot([VTOL.thrust.Time(end); Cruise.airspeed.Time + VTOL.thrust.Time(end)], [VTOL.TransitionStart.Ve.Data(end,1); Cruise.airspeed.Data(:,1)],'black-')
xlim([630 934])
ylabel('Airspeed (m/s)')
xlabel('Time (s)')
plot([714 714], [-20 80],'black:')
plot([773 773], [-20 80],'black:')
plot([914 914], [-20 80],'black:')

figure()
plot(VTOL.TransitionStart.Xe.Time, VTOL.TransitionStart.Xe.Data(:,3)*(-1),'black-')
hold on
plot(Cruise.landingConditions.Xe.Time + VTOL.TransitionStart.Xe.Time(end), Cruise.landingConditions.Xe.Data(:,3)*(-1),'black-')
plot(Cruise.airspeed.Time(end) + VTOL.thrust.Time(end) + Landing.TransitionStart.Ve.Time, Landing.TransitionStart.Xe.Data(:,3)*(-1),'black-')

xlim([630 934])
ylabel('Altitude (m)')
xlabel('Time (s)')
plot([714 714], [-100 600],'black:')
plot([773 773], [-100 600],'black:')
plot([914 914], [-100 600],'black:')

figure()
subplot(3,1,3)
plot(VTOL.TransitionStart.EA.Time, VTOL.TransitionStart.EA.Data(:,3),'black-')
hold on
plot(Cruise.landingConditions.EA.Time + VTOL.TransitionStart.EA.Time(end), Cruise.landingConditions.EA.Data(:,3),'black-')
plot(Cruise.airspeed.Time(end) + VTOL.thrust.Time(end) + Landing.TransitionStart.Ve.Time, Landing.TransitionStart.EA.Data(:,3),'black-')
ylabel('Yaw (rad)')
xlabel('Time (s)')
% 
xlim([630 934])
plot([714 714], [-0.002 0],'black:')
plot([773 773], [-0.002 0],'black:')
plot([914 914], [-0.002 0],'black:')

subplot(3,1,2)
plot(VTOL.TransitionStart.EA.Time, VTOL.TransitionStart.EA.Data(:,2),'black-')
hold on
plot(Cruise.landingConditions.EA.Time + VTOL.TransitionStart.EA.Time(end), Cruise.landingConditions.EA.Data(:,2),'black-')
plot(Cruise.airspeed.Time(end) + VTOL.thrust.Time(end) + Landing.TransitionStart.Ve.Time, Landing.TransitionStart.EA.Data(:,2),'black-')
ylabel('Pitch (rad)')
% 
xlim([630 934])
ylim([-0.25 0.25])
plot([714 714], [-0.25 0.25],'black:')
plot([773 773], [-0.25 0.25],'black:')
plot([914 914], [-0.25 0.25],'black:')

subplot(3,1,1)
plot(VTOL.TransitionStart.EA.Time, VTOL.TransitionStart.EA.Data(:,1),'black-')
hold on
plot(Cruise.landingConditions.EA.Time + VTOL.TransitionStart.EA.Time(end), Cruise.landingConditions.EA.Data(:,1),'black-')
plot(Cruise.airspeed.Time(end) + VTOL.thrust.Time(end) + Landing.TransitionStart.Ve.Time, Landing.TransitionStart.EA.Data(:,1),'black-')

ylabel('Roll (rad)')

ylim([-0.001 0.001])
xlim([630 934])
plot([714 714], [-0.02 0.02],'black:')
plot([773 773], [-0.02 0.02],'black:')
plot([914 914], [-0.02 0.02],'black:')



