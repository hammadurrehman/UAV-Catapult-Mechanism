clear, close all
%% SOLIVING  NON-LINEAR_NON-HOMOGENOUS_SECOND_ORDER_O.D.E
At=pi/4*.0125^2;
n=1.17;
k=1.4;
R=287;
P0=101325;      % atm pressure
P_tank_ini=450000;
T_tank_ini=298.15;
rho_tank_ini=P_tank_ini/R/T_tank_ini;
g=9.8; theta=90*pi/180; 
dpis=.083;       % m diameter of the piston to be used
pis_area=pi/4*dpis^2;    % m2 
P_b=101325;      %Pa    initial pressure in the cylinder,  due to weight on piston
V=0.004;
M_L=1;        %Kg
Tc=298.15;          %intialy in cyl
V0=1e-5;        % piston dead volume
Cd=0.82;
leakage_area=1e-10;     % initial area for the valve, as modeled
B=12;      % viscous damping, in cyl and piston assem
mu=0.045;     % friction
mu_Nc=mu*300;        % representing coulomb friction in piston cyl
mu_Nbr=1850;     % upon our previous prototype tests...
time_step=1.54/36;
time=1.54;
t=0:time_step:time;
x=[0,0,V0,P_b,rho_tank_ini,P_tank_ini,0.0001,leakage_area];      % initial cyl pressure need to be modified with weight
[t,X]=ode45(@(t,x) cat_eq_coupled_v10_valve_area(t,x,k,n,At,R,P_tank_ini,T_tank_ini,rho_tank_ini,pis_area,P0,V,M_L,Tc,g,theta,B,mu,Cd),t,x);

 simOut = sim('pdf_comb_prot_params_test_delete_it','SimulationMode','normal','AbsTol','1e-5',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','yout',...
 'SaveFormat', 'Dataset','ReturnWorkspaceOutputs','on');
sim1=simOut.find('sim1');
sim2=simOut.find('sim2');
sim3=simOut.find('sim3');
sim4=simOut.find('sim4');
sim_time=sim1.time;
sim_pres_tank=sim1.signals.values;
sim_pres_piston=sim2.signals.values;
sim_dist=sim3.signals.values;
sim_vel=sim4.signals.values;

%%  PLOTTING
% errd=abs(sim_dist-X(:,1))./abs(sim_dist);
% errd_sum=sum(errd);
% errd_rms_dist=sqrt(sum(errd.^2));
% subplot(2,1,1)
% plot(t,X(:,1),'-',sim_time,sim_dist,'r')
% title('distance vs time, (m-sec)')
% legend('analytical','simulink')
% xlabel('time')
% ylabel('distance m')
% subplot(2,1,2)
% plot(t,errd.*100)
% ylabel('error %')
% 
figure
errv=(abs(abs(sim_vel)-abs(X(:,2))))./abs(sim_vel);
errv_sum=sum(errv);
errv_rms_vel=sqrt(sum(errv.^2));
subplot(2,1,1)
plot(t,X(:,2),'-',sim_time,sim_vel)
title('velocity vs time, (m/sec-sec)')
legend('analytical','simulink')
xlabel('time')
ylabel('velocity  m/s')
subplot(2,1,2)
plot(t,errv.*100)
ylabel('error %')

%figure
errpis=abs(sim_pres_piston-X(:,4))./abs(sim_pres_piston);
errpis_sum=sum(errpis);
err_rms_Ppis=sqrt(sum(errpis.^2));
subplot(2,1,1)
plot(t,X(:,4),'-',sim_time,sim_pres_piston)
title(['piston pressure vs time, (Pa-sec)  ','n=',num2str(n),'  ','k=',num2str(k)])
legend('analytical','simulink')
xlabel('time')
ylabel('pressure Pa')
subplot(2,1,2)
plot(t,errpis.*100)
ylabel('error %')



% figure
% errp=abs(sim_pres_tank-X(:,6))./abs(sim_pres_tank);
% errp_sum=sum(errp);
% errp_rms_Ptank=sqrt(sum(errp.^2));
% subplot(2,1,1)
% plot(t,X(:,6),'-',sim_time,sim_pres_tank)
% title('tank pressure vs time, (pa-sec)')
% legend('analytical','simulink')
% xlabel('time')
% ylabel('pressure Pa')
% subplot(2,1,2)
% plot(t,errp.*100)
% ylabel('error %')