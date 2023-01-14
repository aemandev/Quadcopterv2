airframe = struct();

g = 9.81;
rho = 1.1;

% Airframe System Specifications
airframe = struct();
% Good for physical quad
Ixx = .0187;
% Ixx = 0.03356;
Iyy = .0134;
% Iyy = 0.03122;
Izz = .04523;
% Izz = 0.05423;

% Body constants
airframe.Cdx = .25;
airframe.Cdy = .25;
airframe.Cdz = .25;
airframe.cDPitch = 7.5*10^-7;
airframe.cDRoll = 7.5*10^-7;
airframe.cDYaw = 7.5*10^-7;
airframe.J = diag([Ixx Iyy Izz]);
airframe.mass = 1.17; % Good for real quad
airframe.larm = .1778; % Good for real quad


% Propeller Constants
airframe.Propeller.Jr = 3.357*10^-7;                                        % Mass moment of inertia of propeller blade
airframe.Propeller.rProp = .128;                                            % Prop radius
airframe.Propeller.Chord = .0080;                                           % Prop chord length
airframe.Propeller.AreaCS = .0034;                                          % Cross-sectional area of propeller
airframe.Propeller.Ctau = 1.140*10^-7;                                      % Yawing moment coefficient


% To tune:
%https://githubmemory.com/repo/ethz-asl/rotors_simulator/issues/634

% airframe.Propeller.Ct = 3.63E-06; %Good for quad    from paper                         % Thrust coefficient
% airframe.Propeller.Ct = 8.54858e-06; % Gazebo value
% airframe.Propeller.Ct = 7.7878e-06; % TEST AND GOT THIS VALUE 
% airframe.Propeller.Ct =6e-06;
airframe.Propeller.Ct = 5.9152e-06; % Calc by hand

% airframe.Propeller.CdProp = 5.11E-08; % Good for quad, Value from paper     % Propeller drag constant
airframe.Propeller.CdProp =6.06428e-05;
% airframe.Propeller.CdProp = .016; % Maybe use this based on Gaz?
% airframe.Propeller.CdProp =8.06428e-05; % Gazebo value