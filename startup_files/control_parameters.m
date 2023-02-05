airframe = struct();

g = 9.81;
rho = 1.1;

% Airframe System Specifications
% airframe = struct();
% Good for physical quad
% Ixx = Simulink.Parameter(single(.0134));
Ixx = Simulink.Parameter(single(4.856*10^-3));
% Ixx = 0.03356;
% Iyy = Simulink.Parameter(single(.0134));
Iyy = Simulink.Parameter(single(4.856*10^-3));
% Iyy = 0.03122;
% Izz = Simulink.Parameter(single(.04523));
Izz = Simulink.Parameter(single(8.801*10^-3));

% Izz = 0.05423;

% Body constants
Cdx = Simulink.Parameter(single(.25));
Cdy = Simulink.Parameter(single(.25));
Cdz = Simulink.Parameter(single(.25));
cDPitch = Simulink.Parameter(single(7.5*10^-7));
cDRoll = Simulink.Parameter(single(7.5*10^-7));
cDYaw = Simulink.Parameter(single(7.5*10^-7));
J = diag([Ixx.Value Iyy.Value Izz.Value]);
% mass = Simulink.Parameter(single(1.17)); % Good for real quad
mass = Simulink.Parameter(single(0.468 ));
% larm = Simulink.Parameter(single(.1778)); % Good for real quad
larm = Simulink.Parameter(single(0.225 ));


% Propeller Constants
% Jr = Simulink.Parameter(single(3.357*10^-7));                                        % Mass moment of inertia of propeller blade
Jr = Simulink.Parameter(single(3.357*10^-5));
% rProp = Simulink.Parameter(single(.128));                                            % Prop radius
rProp = Simulink.Parameter(single(0.0330));
Chord = Simulink.Parameter(single(.0080));                                           % Prop chord length
AreaCS = Simulink.Parameter(single(.0034));                                          % Cross-sectional area of propeller
Ctau = Simulink.Parameter(single(1.140*10^-7));                                     % Yawing moment coefficient


% To tune:
%https://githubmemory.com/repo/ethz-asl/rotors_simulator/issues/634

% Ct = 3.63E-06; %Good for quad    from paper                         % Thrust coefficient
% Ct = 8.54858e-06; % Gazebo value
% Ct = 7.7878e-06; % TEST AND GOT THIS VALUE 
% Ct =6e-06;
% Ct = Simulink.Parameter(single(5.9152e-06)); % Calc by hand
Ct = Simulink.Parameter(single(2.98*10^-6));

% CdProp = 5.11E-08; % Good for quad, Value from paper     % Propeller drag constant
% CdProp = Simulink.Parameter(single(6.06428e-05));
CdProp = Simulink.Parameter(single(0.114*10^-6));
% CdProp = .016; % Maybe use this based on Gaz?
% CdProp =8.06428e-05; % Gazebo value

K = Ct.Value;
L = larm.Value;
C = CdProp.Value;
invControlMat = [-1/(4*K), -1/(4*K*L), -1/(4*K*L), -1/(4*C);
                -1/(4*K),  1/(4*K*L), -1/(4*K*L),  1/(4*C);
                -1/(4*K),  1/(4*K*L),  1/(4*K*L), -1/(4*C);
                -1/(4*K), -1/(4*K*L),  1/(4*K*L),  1/(4*C)];