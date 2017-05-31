clear, close all
%% getting constants and coeffs ready
P_b=101325;
k=1.4;      % specific heat ratio of gases for adiabetic process
R=287;      %j/Kg-k   gen   gas constant 
P_tank_ini=790000;
T_tank_ini=298.15;
rho_tank_ini=P_tank_ini/R/T_tank_ini;        % air density at ambient cond
n=1.105;            % polytropic constant
Cd=0.82;%0.82*iter/5;
At=pi/4*(0.0021)^2;    % m2 area of throat, as modeling flw discharge thr nozz
T0=298.15;      % temperature at stagnant cond. in vessel
V=0.103;          % vessel volume
 % X1 show  one derivative of x
 % X2 shows double derivative of x
 % x1 its the displacement , after integration
 % x2 its the velcoity , after integration
 %X3 shows pressure diffential;

%% SOLIVING  NON-LINEAR_NON-HOMOGENOUS_SECOND_ORDER_O.D.E
time=200;
time_step=7.0175;
t=[0:time_step:time];
x=[P_tank_ini,rho_tank_ini];      % third col   is  P   initial pressure EQUAL TO LOAD CARYING OF PISTON AND STUFF ON IT
                            % fourth is the density for the vessel for
                            % dischage process
[t,X]=ode45(@(t,x) cat_eq_tank_side(t,x,R,k,P_b,At,V,n,P_tank_ini,T_tank_ini),t,x);

 simOut = sim('pdf_1','SimulationMode','normal','AbsTol','1e-5',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','yout',...
 'SaveFormat', 'Dataset','ReturnWorkspaceOutputs','on');
sim1=simOut.find('sim');
%%  PLOTTING

sim_time=sim1.time;
sim_p=sim1.signals.values;

err=abs(sim_p-X(:,1))./abs(sim_p);
err_sum=sum(err);
err_rms=sqrt(sum(err.^2));

plot(t,X(:,1),'b*-',sim_time,sim_p,'ro-')
title(['pressure discharge from the vessel   ','n=',num2str(n),'  ','Cd=',num2str(Cd)])
xlabel('time')
ylabel('pressure Pa')
hold on
yyaxis right
plot(t,err.*100,'--')
legend('from paper vessel charge/discharge','simulink simscape model','error')
ylabel('error %')
