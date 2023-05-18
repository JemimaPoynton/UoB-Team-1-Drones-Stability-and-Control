%% Wing properties
L = 150*9.81;
W = L;
a = 3.724; % This is relatively low
CM0 = -0.02; 

S = 8.34; % [m2] 
b_w = 7; % Wingspan [m]
c = 1.46; % MAC [m]

V_cruise = 25.9; % [m/s]
V_stall = 15.9; % [m/s]
rho = 1.167;

Cl_cruise = 2*L/(rho*S*V_cruise^2);

h0 = 1.97/c; % 1.4 estimate based on inspection of the CW-100, will need to be determined by calculating weights

%% Horizontal Taiplane Properties
AR_w = b_w/c;
AR_h = 5.86; % Sadraey says that the canard should have a higher AR than the wing
dCl = 6.38; % Sectional lift coefficient of tailplane NACA 0009
dwash = 0; % de/da (downwash) 0.2 - 0.4

eta_T = 0.05; 

a1 = dCl/(1 + dCl/(pi*AR_h)); % 2.7 - 4.5
a2 = 2.8822; % 0.8 - 2.5

%% Optimum Tail Arm
Df = 0.48; % Based on saying ~ 10% of the wingspan
Kc = 1.4; % Range between 1 and 1.4 as moving away from a conical shape

V_H = 0.13;
lopt = Kc*sqrt((4*0.9*7.9*V_H)/(pi*Df));

l = 1.87; % Setting L, considering lopt and fuselage limits

%% Stability Margin Requirements
h_vals = linspace(0, 3, 9000);

Kn = 0.08;
Hm = 0.05;

% If the Kn > 0.1
V_ = (h0 - h_vals - Kn)./(a1/a);
figure()
plot(h_vals, V_,'color','black')
hold on

mu = W/(rho*9.81*S*l);

% If Hm > 0.05
V_ = (h0 - h_vals - Hm)./((a1/a) + a1/(2*mu));
plot(h_vals, V_, '--','color','black')

%% Collision avoidance
n = 3.5; % Max collision avoidance manoeuvre accounted for

V_ = Hm*Cl_cruise*n/(a2*(15*pi/180))*ones(size(h_vals));
plot(h_vals, V_, 'color', '#808080','LineStyle',':')

%% If the aircraft should be trimmed with <25 degrees at start of transition
VR = V_stall*1.4; % [m/s]

Cl_R = (2*L/(rho*S*VR^2));
dE = 25*pi/180;

V_ = ((h0 - h_vals).*Cl_R - CM0)/((a1/a)*Cl_R + a1*eta_T + a2*(dE));
plot(h_vals, V_, '-.', 'color', 'black')

Vbar = 0.133
dE = (((h0 - 0.89).*Cl_cruise - CM0)/Vbar - (a1/a)*Cl_R - a1*eta_T)/a2
 

%% Formatting Plots
scatter(1.08, 0.133, 'x', 'black')

xlabel('CG Position h')
ylabel('Tail Volume Coefficient V')

legend('Static Stability Margin 8%', 'Manoeuvre Margin 5%', 'Collision Avoidance @ Cruise', 'Transition Speed', 'Final CG')
ylim([0 0.3])
xlim([0.6 1.2])

%% Re-evaluating tailplane setting
% Happy with the 3 deg angle, to prevent stall etc., may modify based on
% elevator sizing requirements
h = 1.1; % REMEMBER TO CHANGE THIS
hdiff = h0 - h;
Vbar = 0.133;

Cmg = CM0 - (h0 - h)*Cl_cruise + Vbar*((a1/a)*Cl_cruise + a1*eta_T);
eta_T = -(CM0 - (h0 - h)*Cl_cruise + Vbar*((a1/a)*Cl_cruise))/(Vbar*a1);

eta_T_deg = eta_T*180/pi; % < 10 degrees

%% Horizontal Tailplane Verification
Clt = ((h0 - h).*Cl_R - CM0)/Vbar;
eta = (Clt - (a1/a)*Cl_R - a1*eta_T)/a2;

Hm = (h0 - h) - Vbar*((a1/a) + a1/(2*mu));
Kn = (h0 - h) - Vbar*((a1/a));

%% Calculate horizontal tailplane geometry
St = Vbar*S*c/l;
b_h = sqrt(AR_h*St); % Minimise to minimise aerofoil thickness
cm = St/b_h;

taperRatio = 0.9;
ct = (2*taperRatio*cm)/(taperRatio + 1);
cr = ct/taperRatio;

%% Rough weights
t = 0.045*cm; % based on NACA 0009
vol_c = (St)*t;
weight_c = vol_c*2780*0.05;

%% Vertical Tailplane Sizing
% Currently just stability requirements, but additional dynamic
% requirements and spin recovery need to be considered
Kf1 = 0.55; % contribution of the fuselage to the aircraft Cnβ, considering that wingtips will contribute

AR_v = 1.51; 
% AR_v = linspace(1,2,100);

c1 = dCl./(1 + dCl./(pi*AR_v)); % Lift curve slope of vertical tail

sidewash = -0.1; % Sidewash gradient
Cn_beta = 0.08; % Directional stabiltity derivative

V_vert = Cn_beta./(Kf1.*c1.*(1-sidewash)) % Simply rearranging the equation for the stability requirement

lv = 0.88; % distance between the vertical tail aerodynamic center (acv) and the wing/fuselage aerodynamic center
S_winglets = 0.2*2;
l_winglets = 1.05;

V_winglets = S_winglets*l_winglets/(S*c);
V_tail = V_vert - V_winglets;

Sv = V_tail.*S.*b_w./lv
Sv_each = Sv./2;
% 
b_v = sqrt(AR_v.*Sv_each);
c_v = Sv_each/b_v;

vol_v = (Sv).*t;
weight_v = vol_v.*2780*0.05;


%% Check that minimum stability is achieved, even if Kf1 is overestimated
Kf1 = linspace(0.55, 0.9, 1000);
Cn_beta = (Kf1.*c1.*(1-sidewash))*V_vert;

%figure()
%plot(Kf1, Cn_beta) % Can conclude from here that Cn_beta > 0.05 even for minimum Kf1
hold on

Kf1 = 0.55;
sidewash = linspace(-0.3, 0.3, 1000);
Cn_beta = (Kf1.*c1.*(1-sidewash))*V_vert;
%plot(sidewash, Cn_beta) % Can conclude from here that Cn_beta > 0.05, even for the absolute worst case

ct_v = (2*taperRatio*c_v)/(taperRatio + 1);
cr_v = ct_v/taperRatio;