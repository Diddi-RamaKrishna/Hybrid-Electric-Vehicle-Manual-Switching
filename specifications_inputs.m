% This MATLAB script implements a manual switching mechanism for a hybrid electric vehicle (HEV).
% The script allows switching between electric mode, ICE (Internal Combustion Engine) mode, and hybrid mode.

% Load a Simulink example for context
openExample('simscapeelectrical/IPMSMAxleDriveEVExample')
%the example above is used to develop the HEV and a manual switch is implemented
% User input for mode selection
% Enter 1 for Electric mode, 2 for ICE mode, or 3 for Hybrid mode
x = input('Enter: \n 1 for Electric Mode\n 2 for ICE Mode\n 3 for Hybrid Mode: \n');

% Set parameters based on the selected mode
if x == 1
    % Electric mode
    d = 1; % Duty cycle for electric mode
    e = 0.3; % Efficiency or other parameter specific to electric mode
    fprintf('The vehicle is now in Electric Mode:\n')
elseif x == 2
    % ICE mode
    d = 0.3; % Duty cycle for ICE mode
    e = 1; % Efficiency or other parameter specific to ICE mode
    fprintf('The vehicle is now in ICE Mode:\n')
else
    % Hybrid mode
    d = 1; % Duty cycle for hybrid mode
    e = 1; % Efficiency or other parameter specific to hybrid mode
    fprintf('The vehicle is now in Hybrid Mode:\n')
end    
%below sepcifications are taken from bmw i8 version hybrid electric vehicle
% Engine and Motor Parameters
enginepower = 30000; % Power of the engine [W]
PmaxEM = 266000; % Maximum power of the electric motor [W]
TmaxEM = 420; % Maximum torque of the electric motor [N*m]
Ld = 0.00024368; % Stator d-axis inductance [H]
Lq = 0.00029758; % Stator q-axis inductance [H]
L0 = 0.00012184; % Stator zero-sequence inductance [H]
Rs = 0.010087; % Stator resistance per phase [Ohm]
psim = 0.04366; % Permanent magnet flux linkage [Wb]
p = 8; % Number of pole pairs
Jm = 0.1234; % Rotor inertia [Kg*m^2]
Cdc = 0.001; % DC-link capacitor [F]
Vnom = 325; % Nominal voltage [V]

% Battery Parameters
Q = 11600; % Battery capacity [W*hr]
Vbat = 355; % Nominal battery voltage [V]
Ri = 0.001; % Internal resistance [Ohm]
AH = Q/Vbat; % Ampere-hour rating [hr*A]
V1 = 240; % Voltage V1 < Vnom when charge is AH1 [V]
AH1 = AH/2; % Charge AH1 when no-load volts are V1 [hr*A]
SOC0 = 0.7; % Initial State of Charge [%]
AH0 = SOC0*AH; % Initial charge [hr*A]

% ICE Parameters
TmaxICE = 320; % Maximum torque of the ICE [N*m]
PmaxICE = 170000; % Maximum power of the ICE [W]
Km = 178.3; % Gain for ICE model
Tm = 0.1; % Time constant for ICE model [s]

% Control Parameters
Ts = 1e-5; % Fundamental sample time [s]
fsw = 10e3; % Switching frequency for PMSM drive [Hz]
Tsi = 1e-4; % Sample time for current control loops [s]
Tsd = 1e-3; % Sample time for DCDC current control loop [s]

Kp_id = 0.8779; % Proportional gain for id controller
Ki_id = 710.3004; % Integrator gain for id controller
Kp_iq = 1.0744; % Proportional gain for iq controller
Ki_iq = 1.0615e+03; % Integrator gain for iq controller

Kp_ice = 1.65; % Proportional gain for ICE controller
Ki_ice = 3; % Integrator gain for ICE controller

Kp_i = 0.03; % Proportional gain for current control
Ki_i = 5; % Integrator gain for current control

% Zero-Cancellation Transfer Functions
numd_id = Tsi / (Kp_id / Ki_id);
dend_id = [1 (Tsi - (Kp_id / Ki_id)) / (Kp_id / Ki_id)];
numd_iq = Tsi / (Kp_iq / Ki_iq);
dend_iq = [1 (Tsi - (Kp_iq / Ki_iq)) / (Kp_iq / Ki_iq)];

numd_i = Tsd / (Kp_i / Ki_i);
dend_i = [1 (Tsd - (Kp_i / Ki_i)) / (Kp_i / Ki_i)];

% Vehicle Parameters
Mv = 1485; % Vehicle mass [kg]
g = 9.8; % Gravitational acceleration [m/s^2]
rho_a = 1.2; % Air density [kg/m^3]
AL = 2.9; % Max vehicle cross-section area [m^2]
Cd = 0.26; % Air drag coefficient [N*s^2/kg*m]
cr1 = 0.1; % Rolling resistance coefficient 1
cr2 = 0.2; % Rolling resistance coefficient 2
i_t = 9; % Gear reduction ratio
Rw = 0.3; % Wheel radius [m]

% Note: The script integrates an Internal Combustion Engine (ICE) with a hybrid electric vehicle system.
% It enables manual mode switching between electric mode, ICE mode, and hybrid mode to manage power and efficiency.
