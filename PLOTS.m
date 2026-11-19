%% ________________________  UAV X4 SIMULATOR  ____________________________
% -------------------------- SUSBIELLE Pierre -----------------------------
% --------------- piere.susbielle@gipsa-lab.grenoble-inp.fr ---------------
% ----------------------------- GIPSA-Lab ---------------------------------
% ----------------------- Affichages Simulateurs --------------------------

%%
close all

% MAMBO1_DATA = out.MAMBO1_DATA ;
time = MAMBO1_DATA.Time ;
Pos_speed_ref = MAMBO1_DATA.Data(:,1:6) ;
Euler123_ref = MAMBO1_DATA.Data(:,7:9) ;
Thrust_ref = MAMBO1_DATA.Data(:,10) ;
state_feedback = MAMBO1_DATA.Data(:,11:19) ;


x_ref = Pos_speed_ref(:,1) ;
y_ref = Pos_speed_ref(:,2) ;
z_ref = Pos_speed_ref(:,3) ;
x_dot_ref = Pos_speed_ref(:,4) ;
y_dot_ref = Pos_speed_ref(:,5) ;
z_dot_ref = Pos_speed_ref(:,6) ;

phi_ref = Euler123_ref(:,1) ;
theta_ref = Euler123_ref(:,2) ;
psi_ref = Euler123_ref(:,3) ;



sf_x = state_feedback(:,1) ;
sf_y = state_feedback(:,2) ;
sf_z = state_feedback(:,3) ;
sf_x_dot = state_feedback(:,4) ;
sf_y_dot = state_feedback(:,5) ;
sf_z_dot = state_feedback(:,6) ;
sf_phi = state_feedback(:,7) ;
sf_theta = state_feedback(:,8) ;
sf_psi = state_feedback(:,9) ;
 
%%
figure() ;
subplot(311) ; title("Position response in E") ;
plot(time, x_ref) ; hold on ; plot(time, sf_x) ; grid on ; legend("x_{ref}", "x") , xlabel("Time [s]") ; ylabel("Position [m]") ; 
ylim([min(sf_x)-0.2*max(sf_x) max(sf_x)+0.2*max(sf_x)]) ;
subplot(312) ;
plot(time, y_ref) ; hold on ; plot(time, sf_y) ; grid on ; legend("y_{ref}", "y") , xlabel("Time [s]") ; ylabel("Position [m]") ; 
ylim([min(sf_y)-0.2*max(sf_y) max(sf_y)+0.2*max(sf_y)]) ;
subplot(313) ; 
plot(time, z_ref) ; hold on ; plot(time, sf_z) ; grid on ; legend("z_{ref}", "z") , xlabel("Time [s]") ; ylabel("Position [m]") ;
ylim([min(sf_z)-0.2*max(sf_z) max(sf_z)+0.2*max(sf_z)]) ;


figure() ;
subplot(311) ; title("Speed response in E") ;
plot(time, sf_x_dot) ; grid on ; legend("v_{x}") , xlabel("Time [s]") ; ylabel("Position [m]") ; 
ylim([min(sf_x_dot)-0.2*max(sf_x_dot) max(sf_x_dot)+0.2*max(sf_x_dot)]) ;
subplot(312) ;
plot(time, sf_y_dot) ; grid on ; legend("v_{y}") , xlabel("Time [s]") ; ylabel("Position [m]") ; 
ylim([min(sf_y_dot)-0.2*max(sf_y_dot) max(sf_y_dot)+0.2*max(sf_y_dot)]) ;
subplot(313) ; 
plot(time, sf_z_dot) ; grid on ; legend("v_{z}") , xlabel("Time [s]") ; ylabel("Position [m]") ;
ylim([min(sf_z_dot)-0.2*max(sf_z_dot) max(sf_z_dot)+0.2*max(sf_z_dot)]) ;

figure() ;
subplot(311) ; title("Angular response in E (Euler X-Y-Z)") ;
plot(time, phi_ref) ; hold on ; plot(time, sf_phi) ; grid on ; legend("\phi_{ref}", "\phi") , xlabel("Time [s]") ; ylabel("Attitude [rad]") ;
subplot(312) ;
plot(time, theta_ref) ; hold on ; plot(time, sf_theta) ; grid on ; legend("\theta_{ref}", "\theta") , xlabel("Time [s]") ; ylabel("Attitude [rad]") ;
subplot(313) ; 
plot(time, psi_ref) ; hold on ; plot(time, sf_psi) ; grid on ; legend("\psi_{ref}", "\psi") , xlabel("Time [s]") ; ylabel("Attitude [rad]") ;


figure() ;
title("Desired Thrust") ;
plot(time, Thrust_ref) ; hold on ; grid on ; legend("Thrust [N/N]") ;

figure() ;
title("3D Position response in E") ;
plot3(sf_x, sf_y, sf_z) ; hold on ; grid on ; plot3(x_ref, y_ref, z_ref) ; 
legend("Measured trajectory", "Desired trajectory") , xlabel("Position x [m]") ; ylabel("Position y [m]") ; zlabel("Position z [m]") ;

