 close all
%% getting constants and coeffs ready
g=9.8; theta=90*pi/180; 
dpis=.08;       % m diameter of the piston to be used
pis_area=pi/4*dpis^2;    % m2 
G=.1;  % Kg/s  mass flow rate
P0=101325;      %Pa    initial pressure in the cylinder
k=1.4;      % specific heat ratio of gases for adiabetic process
% X1 show  one derivative of x
 % X2 shows double derivative of x
 %X3 shows pressure diffential;
R=287;      %j/Kg-k   gen   gas constant 
M_L=1;        %Kg
%% SOLIVING  NON-LINEAR_NON-HOMOGENOUS_SECOND_ORDER_O.D.E
%mass=0.000147829;
time=10;
time_step=0.1;
t=[0:time_step:time];
x=[0,0,1e-5,101325];      % third col   is  P   initial pressure EQUAL TO LOAD CARYING OF PISTON AND STUFF ON IT
[t,X]=ode45(@(t,x) cat_eq_actuator(t,x,g,theta,pis_area,G,P0,k,R,M_L),t,x);

%%  PLOTTING
sim_time=sim.time;
simp=sim.signals.values;
simvel=sim_vel.signals.values;

errvel=abs(simvel-X(:,2))./abs(simvel);
err_sum_vel=sum(errvel);
err_rms_vel=sqrt(sum(errvel.^2));

errp=abs(simp-X(:,4))./abs(simp);
err_sum_p=sum(errp);
err_rms_p=sqrt(sum(errp.^2));

plot(t,X(:,1),'--')
title('distance vs time')
xlabel('time')
ylabel('distance m')

figure
subplot(2,1,1)
plot(t,X(:,2),'-',sim_time,simvel,'r-*')
title('velocity vs time')
legend('paper Dihovicni Medenica','simulink simscape model')
xlabel('time')
ylabel('velocity  m/s')
subplot(2,1,2)
plot(t,errvel.*100)
ylabel('error %')

figure
plot(t,X(:,4),'b*-',sim_time,simp,'ro-')
title('Pressure change profile'); 
xlabel('time')
ylabel('pressure Pa')
hold on
yyaxis right
plot(t,errp.*100,'--')
legend('paper Dihovicni Medenica','simulink simscape model','error')
ylabel('error %')