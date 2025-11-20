clear % Clear all varibles in the workspace
close all % Close all open figures
clc % Clear the command window


% Setter Link lengths
L1 = 2; L2 = 2; L3 = 0.4; L4 = 1.2;

% ---(standard DH) ---
L(1) = Link('revolute', 'd', 0, 'a', L1, 'alpha', 0);
L(2) = Link('revolute', 'd', 0.4, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', -0.4, 'a', L3, 'alpha', 0);
L(4) = Link('prismatic', 'a', L4, 'alpha', -pi/2, 'theta', 0, 'offset', 0);

% Lager Serialchain til Armen
FL_robot = SerialLink(L, 'name', 'ForkLift Robot');

% Setter grenser på leddvinklene
FL_robot.qlim = [deg2rad([0 120]); deg2rad([-160 0]); [-pi/2 pi/2]; [0 0.75]];

% --- Liten base under roboten (20 cm) ---
base_height = 0.40;                   % 20 cm
FL_robot.base = transl(0, 0, base_height) * trotx(pi/2);


%Definerer workspace
W = [-2 8 -2 5 -2 5];

mask = [1 1 1 0 1 0];

%Demonstrasjon av bevegelse:

%velger punkt
qstart = deg2rad([90 -160 70 0]);
qpall = deg2rad([10 -40 30 0]);

Tstart = FL_robot.fkine(qstart).T;
Tpall = FL_robot.fkine(qpall).T;        %transformasjonsmatrise til pallposisjon
Tfront = Tpall * transl(-1.5, 0, 0);                    % Lager et punkt foran pallen                   
Tloft = Tfront * transl(0, 0, 1);

% Finn en leddkonfigurasjon for punktet rett foran pallen
qfront = FL_robot.ikine(Tfront, 'q0', qstart, 'mask', mask);

%% 1) Start -> legg ned pall (joint space motion)
q_traj_1 = jtraj(qstart, qpall, 30);

%% 2) Lineært ut fra pall (kartesisk)
T_traj_ut = ctraj(Tpall, Tfront, 20);
q_traj_2  = FL_robot.ikine(T_traj_ut, 'q0', q_traj_1(end,:), 'mask', mask);

%% 3) tilbake til start (cartesian motion)
T_traj_opp = ctraj(Tfront,Tloft, 20);
q_traj_3 = FL_robot.ikine(T_traj_opp, 'q0', q_traj_2(end,:), 'mask', mask);

%% 4) tilbake til start (cartesian motion)
q_traj_4 = jtraj(q_traj_3(end,:), qstart, 15);

%% Sett sammen
q_all = [q_traj_1;
         q_traj_2;
         q_traj_3;
         q_traj_4];
q_all(:,4) = 0;
