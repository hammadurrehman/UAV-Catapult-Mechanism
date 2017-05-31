 clear, close all
%% CAD PARAMS
tr_width=1;             %m
tr_len=.5;              %m
rail_ht=0.003;          %m
rail_len=15;            %m
rail_width=1;           %m
trol_high=0.5;          %m
%mass=200;               %Kg
alpha=11;               %deg
rail_fric=0.2;   
g=9.81;                 % gravity accel
D=200;                  %N,  air drag force on trolley (any data from anywhere)
ext_frc_trly=15;      % using as trolley  plus   UAV   weigth, put in density of 1 m3 block

%% atmoshperic conditions: (thermophysical protperties from INCORPERA)
P_0=101325;
T_0=298.15;
h=100;          % air convective heat trans coeff
atm_mass=9999999999;    % assumed big no.
atm_C=1033;      % for ambient air..,  

%% pressure accumulator model (m-k-s)
acc_L=2.5;              % assuming cylinderical vertical tank, (25 liter)
acc_D=0.078;
k_acc=80;           
wall_th_acc=0.005;
wall_area_acc=pi*acc_D*acc_L;
acc_vol=acc_L*pi/4*acc_D^2;
rho_acc=7800;       % material desnsity of the pressure vessel,
C_acc=447;          % acc material's  specific heat coeff
%initial conditions:-   atmospheric

%% air compressor model params (m-k-s)
comp_fl=.55;

%% pipe model params (m-k-s) %initial conditions:-   atmospheric
pipe_len=1;
pipe_dia=2*25.4/1000;   %m   pipe dia, stds..,  in inches , converted to m, (1,1.25,1.5 ...)inches
eq_len_resistance=0;      %now we need to study and find std lengths from any resource.
int_surf_rough=1e-5;       % m , find from any resource
pipe_wall_thk=0.001;
rho_pipe=7800;              %Kg/m3 pipe material density
k_pipe=80;                 %N/m-K pipe mat cond coef

%%valve model params (m)  %initial conditions:-   atmospheric
Cd=0.92;
valve_dia=pipe_dia;
leakage_area=1e-15;       % x0   depicts piston movement, minimal area
max_orf_area=pi/4*pipe_dia^2;

%% actuator_piston model params (m) %initial conditions:-   atmospheric
mass_pis=0.5;
% brkwy_fric=110;     % breakaway friction force
% col_fric=10;       % N
% fric_damp=100;      % N/m/s
fric_damp=10;      % viscous damping, in cyl and piston assem
mu=0.045;     % friction
col_fric=mu*100;        % representing coulomb friction in piston cyl
brkwy_fric=5;     % upon our previous prototype tests...
mass_act=3;             %mass of whole actuator
k_cond_act=100;             %N/m2-K
L_str=2.2;            % actuator length, assuming full length rod less
dp=2.9*25.4/1000;               % converted to area in simulink piston model
wall_area_act=pi*dp*L_str;
wall_th_act=0.001;      %  m wall thickness
rho_act=7800;           % material density actuator
C_act=447;              % act  material specific heat coeff
V0=1e-5;                % actuator dead volume
% initial conditions (m-k-s)
x0=0;

time=1;    %sec
time_step=0.01; %sec
iter=1;
%%  MONTE-CARLO-SIM-WORK    (m-Kg-s)
% Pmax=250*101325;
% Pmin=10*101325;
comp_fl_max=0.65;                      % air compressor as installed to charge the accumulator
comp_fl_min=0.5;
dp_min=78/1000;                    % piston diameter in,   m
dp_max=78/1000;
L_str_max=3;                       % length of stroke, as if just piston, no rope mult.., m
L_str_min=1.5;
V0_min=1e-4*1e-9;                   % this piston initial volume , that has ebought gas to sustain trolley and payload, mass
V0_max=1e-3*1e-9;
pipe_len_max=1;                     % piep lenght , this pipe will be laid down in structre,  m
pipe_len_min=0.5;
pipe_dia_max=2*25.4/1000;         % inch pipe dia converted to meter
pipe_dia_min=1.5*25.4/1000;
max_orf_area_min=pi/4*pipe_dia_min^2;         % m2,  this is orifice of valve that is connected to the accumulator
max_orf_area_max=pi/4*pipe_dia_max^2;
alpha_max=12;                       % its the aerodynamic launch angle of the UAV., in degrees
alpha_min=10;
acc_vol_max=0.03;                      % pressure accumulator volume parameterization for manageble gas energy storage.
acc_vol_min=0.01;                    % m3

                        %% initialization
                        loop=100;
                        dp=dp*ones(loop,1);
                        L_str=L_str*ones(loop,1);
                        V0=V0*ones(loop,1);
                        max_orf_area=max_orf_area*ones(loop,1);
                        comp_fl=comp_fl*ones(loop,1);
                        acc_vol=acc_vol*ones(loop,1);
                        alpha=alpha*ones(loop,1);
                        pipe_len=pipe_len*ones(loop,1);
                        pipe_dia=pipe_dia*ones(loop,1);                      
                        cost_func=ones(loop,1);

for iter=1:loop
    %P0(iter)=(Pmax-Pmin)*rand+Pmin;                                                %press source
    %comp_fl(iter)=(comp_fl_max-comp_fl_min)*rand+comp_fl_min;                         %mass flow of compressor
    %pipe_len(iter)=(pipe_len_max-pipe_len_min)*rand+pipe_len_min;
    %pipe_dia(iter)=(pipe_dia_max-pipe_dia_min)*rand+pipe_dia_min;
     %dp(iter)=(dp_max-dp_min)*rand+dp_min;                                          %piston dia
     L_str(iter)=(L_str_max-L_str_min)*rand+L_str_min;                              % piston len
    %V0(iter)=(V0_max-V0_min)*rand+V0_min;                                          % piston dead vol
    %max_orf_area(iter)=(max_orf_area_max-max_orf_area_min)*rand+max_orf_area_min;  % valve of accumulator area opening
%    alpha(iter)=(alpha_max-alpha_min)*rand+alpha_min;
     acc_vol(iter)=(acc_vol_max-acc_vol_min)*rand+acc_vol_min;                         % pressure accumulator params
    simOut = sim('nediep_new_params_UAV_final',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','yout',...
             'timeout', 60,...
             'SaveFormat', 'Dataset','ReturnWorkspaceOutputs','on');
out(iter).force_on_tr=simOut.find('force_on_trolley');
out(iter).pos_tr=simOut.find('pos_trolley');
out(iter).velocity=simOut.find('velocity');
out(iter).acceleration=simOut.find('accel');
out(iter).act_pres=simOut.find('act_pres');        %direct pressure on the piston as measured
out(iter).acc_pres=simOut.find('acc_pres');        %accumulator pressure as measured
%out(iter).mass_fl_piston=simOut.find('mass_fl_to_piston');
%out(iter).pres_to_piston=simOut.find('pres_to_piston');
a=10; b=0; c=1; d=10;     % weightage
cost_func(iter)=a/(max(out(iter).velocity)-15)^2+...
             b/dp(iter)^2+...
             c*acc_vol(iter)^2+...
             d*L_str(iter)^2;
             %a*(max(abs(out(iter).pos_tr))/8)^2+...
             %max(abs(out(iter).velocity))+...
             %max(abs(out(iter).acceleration))+...
             %1/max(abs(out(iter).force_on_tr))+...
%pause(10);      % pause in seconds
iter
end
plot(cost_func)
save('31-5-2017_final_thesis_run1')

% 
% t=1:36;
% plot(t*0.05,pres_gr(:,1),'o-','LineWidth',1.5)
% ylabel('guage pressure Pa')
% xlabel('time secs')
% hold on
% plot(t*0.05,pres_gr(:,2),'*-','LineWidth',1.5)
% hold on
% plot(t*0.05,pres_gr(:,4),'--','LineWidth',1.5)
% hold on
% yyaxis right
% plot(t*0.05,pres_gr(:,3),'LineWidth',1.5)
% ylabel('error')
% legend('prototype ','simscape','analytical','error')
% 
% 
% figure 
% plot([0:0.01:1],velocity,'LineWidth',1.5)
% ylabel('velocity m/s')
% xlabel('time secs')
% figure
% plot([0:0.01:1],act_pres,'LineWidth',1.5)
% ylabel('absolute pressure Pa')
% xlabel('time secs')