%% Properties
hfwd = 0.895; % most forward centre of gravity
haft = 1.09;
c = 1.46; % [m]
b = 7;
S = 8.34;

lv = 0.88; % [m] distance between the vertical tail aerodynamic center (acv) and the wing/fuselage aerodynamic center
l_winglets = 1.05;

Sv = 0.466; % [m^2]
cv = 0.3941; % [m]
Vv = 0.0072; % VERTICAL TAIL ONLY

S_winglets = 0.238*2;

V_tot = 0.0482;
S_tot = Sv + S_winglets;
b_tot = 1.0911; % Total span on 1 winglet and 1 vertical stabiliser

Sf = 1.127; % [m^2];
Vs = 15.9; % [m/s]

Ss = Sv + Sf; % [m^2] projected side area

%% Identify most unfavourable conditions
lvt = lv - (haft - hfwd); % [m]
rho = 1.167;

%% Calculate airspeeds
U1 = 1.4*Vs; % REVISED RESTRICTION ON TRANSITION SPEED
Vw = 13; % [m/s]

VT = sqrt(U1^2 + Vw^2); % [m/s] Total airspeed

%% Determine centre of the projected side area
xf = 3.3/2;
xv = (3.3 - cv) + 3.3/2;

xca = (Sf*xf + Sv*xv)/(Ss);

xcg = haft*c;
dc = xca - xcg;

%% Calculate cross-wind force, sideslip angle, and sideslip derivatives
CDy = 0.4; % Side drag coefficient (TBD)
Fw = 0.5*rho*(Vw^2)*Ss*CDy;

beta = atan(Vw/U1);

spanRatio = 0.55; % setting span ratio as 0.45 to be equal to 0.45 of the total span between winglets and stabiliser
chordRatio = 0.45;
tr = 0.6; % As read from graph in reference

Kf1 = 0.55;
Kf2 = 1.5; % contribution of the fuselage to the derivative Cyβ

c1 = 2.71; % vertical tailplane lift coefficient
d_sigma = 0.1;
eta_v = 0.90; % dynamic pressure ratio

Cnb = Kf1*c1*(1 - d_sigma)*(V_tot); % rate of change of yawing moment with sideslip angle
Cyb = -Kf2*c1*(1 - d_sigma)*eta_v*(S_tot/S);

%% Calculate aircraft control derivatives
Cydr = c1*eta_v*tr*spanRatio*(S_tot/S);
Cndr = -c1*V_tot*eta_v*tr*spanRatio;

%% Calculate required rudder deflection
Cn0 = 0;
Cy0 = 0;

% Solving simultaneous equations
Q1 = 0.5*rho*VT^2;
Q2 = 0.5*rho*Vw^2;

sigma = linspace(-0.5236, 0.6236, 90000);

dr = (-Cn0 - Cnb*(beta - sigma) - (Fw*dc*cos(sigma))./(Q1*S*c))*(1/Cndr);
plot(sigma*180/pi,dr*180/pi,'black')
grid on
hold on

dr = (Fw/(Q1*S) - Cyb*(beta - sigma))*(1/Cydr);
plot(sigma*180/pi,dr*180/pi, 'black--')

xlabel('Sideslip angle [deg]')
ylabel('Rudder deflection [deg]')
legend('Equation 1', 'Equation 2')

%% Check spin recovery is met
alpha = 40*pi/180; % assume that the aircraft is spinnning at max 40 deg

Ixx = 107; % [kgm^2]
Izz = 196; % [kgm^2]
Ixz = 0.69; % [kgm^2]

transform = [cos(alpha)^2      sin(alpha)^2    -sin(2*alpha);
             sin(alpha)^2      cos(alpha)^2     sin(2*alpha);
             0.5*sin(2*alpha) -0.5*sin(2*alpha) cos(2*alpha)];

I_w = transform*[Ixx; Izz; Ixz]; % Moment of inertia in the wind axis
RSR = 1.05; % [rad/s^2]

NSR = ((I_w(1)*I_w(2) - I_w(3)^2)/I_w(1))*RSR;

Sve = 0.85*S_tot;
Vve = V_tot*1;

Cndr_e = -c1*Vve*tr*spanRatio;
dr_spin = (2*NSR)/(rho*Vs^2*S*b*Cndr_e); % [rad]
dr_spin_deg = dr_spin*180/pi % Well below max deflection

%% Check that a coordinated turn can be achieved
% Needs to be solved concurently with the ailerons

dA = linspace(0, 0.5, 9000); % assume a max deflection of 30 deg in a turn
Rt = 4583; % [m] radius of turn
U1 = 25.49; % [m/s]
Q = 0.5*rho*U1^2;

Cyr = 0.3750;
Cyda = 0.03;

% figure()
% dr = -(1/Cydr).*(Cyb*beta + Cyr*(Rt*b/(2*U1)) + Cyda.*dA);
% plot(dA, dr)
% Plot Equations 12.130 - 12.132 and find intercept dr once stability
% derivatives have been obtained

%% Calculate dimensions
c_r = cv*chordRatio;
b_r = spanRatio*b_tot; % Per rudder
A_r = c_r*b_r;

taperRatio = 0.9;

ct_r = (2*taperRatio*c_r)/(taperRatio + 1);
cr_r = ct_r/taperRatio;