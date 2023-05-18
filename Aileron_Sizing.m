%% Requirements and preliminary ratios
phi = 30*pi/180; % Required bank angle for MIL-STD of light aircraft
t2_req = 1.3; % [s]

bai_ratio = 0.65; % inboard position of aileron,
bao_ratio = 0.95; % outboard position of aileron, ...
ba_ratio = 0.3; % allows a 30% aileron, with a small 0.05b gap from the wingtip
chordRatio = 0.25; % based on very rough estimate of rear spar location

ta = 0.46; % read from graph based on chord ratio

%% Properties
taperRatio_w = 0.33;
b = 7;
c = 1.46;
cr_w = 1.5; % [m]
a = 3.82;
Vs = 15.9; % [m/s]
rho = 1.167;
S_tot = 9.43; % [m^2]
Ixx = 107.3; % This design is valid for Ixx < 200
S = 8.34;

%% Calculate aileron rolling moment coefficient derivative
yi = bai_ratio*b/2; % inboard position (semi-span)
yo = bao_ratio*b/2;

Clda_t1 = ((2*a*ta*cr_w)/(S*b)); % Clda equation term 1
Clda_t2 = yo^2/2 + (2/3)*((taperRatio_w-1)/b)*yo^3; % """ 2
Clda_t3 = yi^2/2 + (2/3)*((taperRatio_w-1)/b)*yi^3; % """ 3

Clda = Clda_t1*(Clda_t2 - Clda_t3);

%% Calculate aircraft rolling moment at max deflection
da_max = 20*pi/180; % max aileron deflection
C1 = Clda*da_max; % aircraft tolling moment coefficient

VR = 1.4*Vs;
LA = (0.5*rho*VR^2)*S*C1*b;

%% Calculate steady-state roll rate
yD = 0.4*b/2; % Drag is assumed to be at 40% of the wingspan
CDr = 0.9; % Average value

Pss = sqrt((2*LA)/(rho*(S_tot)*CDr*yD^3)); % [rad/s]
phi1 = Ixx*log(Pss^2)/(rho*(yD^3)*S_tot*CDr); % bank angle at which steady state is achieved (this should be high)

%% Calculate time to achieve 30 deg bank angle
P_dot = Pss^2/(2*phi1);

t2 = sqrt(2*phi/P_dot); % [s] Check that this is less than 1.3

%% Calculate final geometry
c_a = c*chordRatio; % mean chord, chord will follow the wing taper ratio
b_a = yo - yi; % per aileron
A_a = c_a*b_a; % per aileron

ct_a = (2*taperRatio_w*c_a)/(taperRatio_w + 1);
cr_a = ct_a/taperRatio_w;

%% Plot bank angle against time
phi = linspace(0, 80, 1000);
t2_vals = sqrt(2.*(phi*pi/180)./P_dot);

figure()
plot(t2_vals, phi, 'black')
grid on
xlabel('Time (s)')
ylabel('Bank angle (deg)')
