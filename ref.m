%% -------------------------- MOBILE ROBOTICS: UAVs -----------------------
% ----------------------- Lab session on aerial vehicles ------------------
% ----------- Author: Pierre Susbielle, pierre.susbielle@gipsa-lab.fr -----


clear all
close all


%% I. Simulation Parameters 

Ts = 0.01 ;     % [s] Sample time 
m = 0.075 ;     % [kg] Drone mass
g = 9.81 ;      % [m.s-2] Gravity acceleration

GLOBAL_PARAMETERS = [Ts m g] ;
initial_pos_conditions_M1 = [0 0 1]' ;


%% II. Parot MAMBO drone model (simulation) parameters

ekf_model = c2d(tf(1, [0.01 1]), Ts) ; % EKF model

delay = 0.08 ;          % [s] Communication delay [s]
Tmin = 0.4786 ;         % [N] Saturation in T = 1*0.0981/0.37 //T=0.0981/0.37*uT+0.7437 
Tmax = 1.0088;          % [N] Maximal thrust


% ------ Dynamic model of thrust 
Ac_thrust = [0.92] ;
Bc_thrust = [0.04 1] ;
sysc_thrust = tf(Ac_thrust, Bc_thrust) ;
sysd_thrust = c2d(sysc_thrust,Ts) ;
B_thrust = sysd_thrust.num ;      % BT{1} numerator of drone pitch/roll model 
A_thrust = sysd_thrust.den ;      % AT{1} den of drone pitch/roll model
%step(sysc)

% ------ Dynamic model of yaw
Ac_yaw = [1] ;
Bc_yaw = [0.4 1] ;
sysc_yaw = tf(Ac_yaw,Bc_yaw) ;
sysd_yaw = c2d(sysc_yaw, Ts) ;
B_yaw = sysd_yaw.num ;       % B{1} numerator of drone yaw model 
A_yaw = sysd_yaw.den ;       % A{1} den of drone yaw model
%step(sysc)


% ------ TO DO : Dynamic model of control angles
% ------ Dynamic model of roll
delay_roll = 0.13;
b_roll = 0.02;
ksi_roll= 0.7;
wn_roll= 22;
roll_min = -0.25;
roll_max = 0.25;
Ac_roll = [1] ;
Bc_roll = [1/wn_roll^2 2*ksi_roll/wn_roll 1] ;
sysc_roll = tf(Ac_roll,Bc_roll) ;
sysd_roll = c2d(sysc_roll, Ts) ;
B_roll = sysd_roll.num ;       % B{1} numerator of drone yaw model 
A_roll = sysd_roll.den ;       % A{1} den of drone yaw model
%step(sysc)

% ------ Dynamic model of pitch
% ksi_pitch=
% wn_pitch=
% Ac_pitch = [1] ;
% Bc_pitch = [1/wn_pitch^2 2*ksi_pitch/wn_pitch 1] ;
% sysc_pitch = tf(Ac_pitch,Bc_pitch) ;
% sysd_pitch = c2d(sysc_pitch, Ts) ;
% B_pitch = sysd_pitch.num ;       % B{1} numerator of drone yaw model 
% A_pitch = sysd_pitch.den ;       % A{1} den of drone yaw model
%step(sysc)


%% IV. Position and Speed control
%% IV.1 Position and Speed LQR control 
% II.I LQR Speed & position Loop - [pos speed] // 1/s² model assomption


% ------ TO DO: Declare linear approximated model for pos-speed loop
% Change the following variabble to adapt to the problem
A_pos_speed = [ zeros(3,3),  eye(3) ;
                zeros(3,3),  zeros(3,3) ];
% for psi=0
B_pos_speed = [zeros(3,3);
                0 g 0;
               -g 0 0;
               0 0 1/m];

% C_pos_speed = [eye(3) zeros(3,3)];
% D_pos_speed = [zeros(3,3)];
C_pos_speed = [eye(6)];
D_pos_speed = [zeros(6,3)];

sys = ss(A_pos_speed, B_pos_speed, C_pos_speed, D_pos_speed);

% ------ TO DO: discretize the problem 
sys_d = c2d(sys, Ts);

Ad_pos_speed = sys_d.A;
Bd_pos_speed = sys_d.B;

Cd_pos_speed = sys_d.C;
Dd_pos_speed = sys_d.D;

% Feedback control scheme
% B(psi,g,m) = B(g,m)*R(psi)
B_lin = [zeros(3,3);
           0 g 0;
          -g 0 0;
           0 0 1/m];

sys_2 = ss(A_pos_speed, B_lin, C_pos_speed, D_pos_speed);
sys_d_2 = c2d(sys_2, Ts);

Ad_2 = sys_d_2.A;
Bd_2 = sys_d_2.B;

Cd_2 = sys_d_2.C;
Dd_2 = sys_d_2.D;


% ------ TO DO : Check feasibility and find a control law for position and
% speed
R = eye(3);
Q = 1*eye(6);
% Q = [5*eye(3) zeros(3,3);
%     zeros(3,3) eye(3)];
K_lqr = dlqr(Ad_2, Bd_2, Q, R)

%% IV.2 Position and Speed Integral control 

% ------ TO DO : Check feasibility and find an integral control law for position and
% speed
A_aug = [ zeros(3,3),  eye(3),    zeros(3,3) ;
          zeros(3,3),  zeros(3,3),zeros(3,3) ;
         -eye(3),      zeros(3,3),zeros(3,3) ];
B_aug = [B_pos_speed ;
          zeros(3,3) ];

sys_aug_c = ss(A_aug, B_aug, eye(9), zeros(9,3));

sys_aug_d = c2d(sys_aug_c, Ts, 'zoh');

Ad_aug = sys_aug_d.A;
Bd_aug = sys_aug_d.B;

Q_aug = blkdiag(Q, 1*eye(3));
R = 10*eye(3);

K_aug = dlqr(Ad_aug, Bd_aug, Q_aug, R)

Kx = K_aug(:,1:6);
Ki = K_aug(:,7:9);

% u = -Kx*x - Ki*z;

Co = ctrb(A_aug,B_aug);
rank(Co) % =9 so it's ctrb