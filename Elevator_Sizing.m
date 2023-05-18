%% Properties
Iyy = 93.7; % [kgm^2] This requires the maximum Clc for Iyy < 20 kgm^2

%% Inital Sizing for Rotational Requirements
theta_dd = 10*pi/180; % [deg/s^2] Angular acceleration
elevator = -25*pi/180; % [rad]
Cl0 = 0.107; % Wing lift coefficient at zero AoA
CM0 = -0.02; % Zero lift pitching moment
a = 3.72;

rho = 1.167; % [kg/m^3]

Vstall = 15.9; % [m/s]
VR = 1.4*Vstall; % Rotation speed (speed at TO)

Q = 0.5*rho*VR^2; % Dynamic pressure

S = 8.34; % [m^2]
St = 0.824; % [m^2]
c = 1.46; % [m]

h0 = 1.97/c;
hfwd = 0.95; % most forward centre of gravity
haft = 1.1;

etaT = 0.047; % Canard incidence (proposed)
alpha = 0; % AoA at rotation point

Lwbn = Cl0*Q*S;
M0 = CM0*Q*S*c;

T_tot = 150*9.81; % Total thrust required
thrustRatio = 1.4;

% Tf = T_tot/((1 + thrustRatio)*2);
% Tr = Tf*thrustRatio
Tf = 0; % Thrusts assumed to be balanced
Tr = 0;

lc = 1.89;  % [m] Canard moment arm (ac)
lfr = 1.95; % [m] Front rotors (x2) moment arm (ac)
lrr = 1.375; % [m] Rear rotors (x2) moment arm (ac)

xcg_c = (lc - (h0 - hfwd)*c); % [m] Distance between cg (of a/c) and ac of canard
xcg_fr = lfr - (h0 - hfwd)*c;
xcg_rr = lrr + (h0 - hfwd)*c;

Lc = (1/xcg_c)*(Iyy*theta_dd + ... % [N] Lift required from canard
      + Lwbn*(h0 - hfwd)*c ...
      - M0 ...
      - 2*Tr*xcg_rr ...
      + 2*Tf*xcg_fr); 

Clc = Lc/(Q*St) % Required Cl

alpha_h = alpha + etaT;
a1 = 4.73;

te = (Clc/a1 + alpha_h)/elevator; % Effectiveness parameter
chord_ratio = 0.46; % From reading graph of te
b = 1.95;

cm = 0.4135; % Mean chord length of the canard
cr = 0.4353; % Root ""
ct = 0.3918; % Tip ""

c_c = [cm; cr; ct];
span_ratio = 0.7; 

be = 1.9932*span_ratio;
ce = chord_ratio*c_c; % Chord length of the elevator [mean; root; tip]


%% Checking horizontal tail lift coefficient with max elevator deflection
d_alpha_oe = -1.15*chord_ratio*(-15); % [deg]
% Run Prandtl_Check_Lift then compare with Clc to see if they match

%% Calculate elevator effectiveness derivatives
% Used to confirm that the max deflection required never goes beyond the
% established value
V_H = 0.136;
te_2 = 0.64; % Effectiveness parameter as re-evaluated for the revised chord ratio
l = 1.89;

C_mde = a1*V_H*(span_ratio)*te_2 % [/rad] change of aircraft pitching moment coefficient with respect to elevator deflection
C_lde = a1*(St/S)*span_ratio*te_2 % [/rad] rate of change of aircraft lift coefficient with respect to elevator deflection
C_lhde = a1*te_2; % [/rad] rate of change of tail lift coefficient with respect to elevator deflection [a2] 

%% Calculate elevator deflection and plot for different conditions
% Assuming transition has been taken care of
% Considering cruise/loiter altitude, and transition altitude

zT = 0; % Assuming that thrust and drag have no moment
T = 1e3;
V = linspace(22.26, 45, 1000); % [m/s]
Cl0 = 0.107;

rho = 1.167; % Air density
Q = 0.5*rho*V.^2; % Dynamic pressure

Cl = 150*9.81./(Q*S);
thrustEffect = T*zT./(Q*S*c); % Placeholder

h = [hfwd haft]; 

Cma = a.*(h - h0) + a1*V_H;
% 
dE_fwd = (((h0 - h(1)).*Cl - CM0)/V_H - (a1/a)*Cl - a1*etaT)/(C_lhde + V_H*(a1/a)*C_lde +(h0-h(1))*C_lde - C_mde);
dE_aft = (((h0 - h(2)).*Cl - CM0)/V_H - (a1/a)*Cl - a1*etaT)/C_lhde;
dE_aft = (((h0 - h(2)).*Cl - CM0)/V_H - (a1/a)*Cl - a1*etaT)/(C_lhde + V_H*(a1/a)*C_lde +(h0-h(2))*C_lde - C_mde);



figure()
plot(V, dE_fwd*180/pi, 'black')
hold on
plot(V, dE_aft*180/pi, 'black--')

grid on
xlabel('Airspeed [m/s]')
ylabel('Elevator deflection for trim [deg]')
legend('Forward CG', 'Aft CG')
