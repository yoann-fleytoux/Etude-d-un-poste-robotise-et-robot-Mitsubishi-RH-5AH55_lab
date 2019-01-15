% Mod�lisation du robot SCARA Mitsubishi

%% NE PAS MODIFIER CETTE PARTIE
clc
clear all
close all

% D�finition des param�tres g�om�triques du robot
a1 = 0.4; % Distance entre L1 et L2 en m
a2 = 0.4; % Distance entre L2 et L3 en m
L3 = 0.2; % Distance entre la cam�ra et la 3�me liaison (point O3) en m
h = 0.8;  % Hauteur au-dessus du plan de travail en m

% Mesures renvoy�es par la cam�ra (position et orientation):
xP = 0.5; yP = 0.2; thetaP = pi/2

% On d�finit ici 4 liaisons 3 rotoides et 1 prismatique (la 3�me) � l'aide
% des param�tres de Denavit/Hartenberg.
% Attention les param�tres de Denavit Hartenberg ne correspondent pas  aux
% conventions du cours

% theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');
L(3) = Link([ 0     0   0  0 1]);
L(4) = Link([ 0     0   0  0], 'standard');

% D�finition des limites physiques des axes (en radians)
L(1).qlim = [-pi pi];
L(2).qlim = [-pi pi];
L(3).qlim = [-2 0];
L(4).qlim = [-pi pi];

% On d�finit ici notre robot comme la concat�nation des liaisons
% pr�c�dentes, on lui donne un nom, etc.
robot = SerialLink(L, 'name', 'Mitsubishi', 'comment', 'from AIP');

%robot.teach() : ne pas utiliser cette m�thode car risque de bug

% Diff�rentes configurations de test pour les MGD/MGI
Q1 = zeros(1,4);
Q2 = [pi/2 0 0 0];
Q3 = [0 pi/2 0 0];
Q4 = [0 0 0 pi/2];
Q5 = [-pi/2 0 0 0];
Q6 = [0 0 -0.5 0];
Q7 = [-pi/2 pi/2 -0.5 0];



% FIN DE LA PARTIE NON MODIFIABLE

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TP Etudiant
Q=Q7;

%% Calcul du MGD
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);

T01=[cos(q1),-sin(q1),0,0;
    sin(q1),cos(q1),0,0;
    0,0,1,0;
    0,0,0,1];

T12=[cos(q2),-sin(q2),0,a1;
    sin(q2),cos(q2),0,0;
    0,0,1,0;
    0,0,0,1];

T23=[1,0,0,a2;
     0,1,0,0;
    0,0,1,q3;
    0,0,0,1];

T34=[cos(q4),-sin(q4),0,0;
    sin(q4),cos(q4),0,0;
    0,0,1,0;
    0,0,0,1];


T04=T01*T12*T23*T34;
robot.fkine(Q)



%On obtient les m�me resultats

% A COMPLETER
% AFFICHER LA MATRICE DE PASSAGE T04 ENTRE LE REPERE R0 ET LE REPERE OUTIL

robot.plot(Q)
hold on
trplot(T04,'frame', '4') % afficher le rep�re outil (via MGD) sur la figure


%% Affichage sur la figure du rep�re cam�ra
% Calculer ici TOC la matrice de passage homog�ne entre R0 et Rc
T23_camera=[1,0,0,a2+L3;
     0,-1,0,0;
    0,0,-1,0;
    0,0,0,1];

T0C=T01*T12*T23_camera;

% A COMPLETER

hold on % permet de superposer plusieurs courbes sur une seule figure
view
trplot(T0C,'frame', 'C') %afficher sur la figure


 %% D�termination de la position et de l'orientation de la pi�ce dans R0
  % Donn�es: position et orientation de la pi�ce dans Rc (rep�re cam�ra)
  PosPieceDsRc = [xP;yP;h;thetaP];
  %T02_Q1 ->TOC la matrice de passage homog�ne entre R0 et Rc
  TC0=inv(T0C)
  PosPieceDsR0 = TC0*PosPieceDsRc;
  
  
  % Calculer et afficher la position (X,Y,Z) de la pi�ce dans R0
  % A COMPLETER
 
  plot3(PosPieceDsR0(1),PosPieceDsR0(2),PosPieceDsR0(3),'ro'); % pour afficher
  
  % Calculer la position de l'objet dans R4 (rep�re outil)
  % A COMPLETER
  %PosPieceDsR4=T34*PosPieceDsRc;
  
 % plot3(PosPieceDsR4(1),PosPieceDsR4(2),PosPieceDsR4(3),'rc'); % pour afficher
  
  % En d�duire la matrice de passage T0P entre le rep�re R0 et l'objet
  % A COMPLETER
  T0P=[cos(thetaP),sin(thetaP),0,PosPieceDsR0(1);
    sin(thetaP),-cos(thetaP),0,PosPieceDsR0(2);
    0,0,1,PosPieceDsR0(3);
    0,0,0,PosPieceDsR0(4)];

  

  %% Calcul des deux solutions du MGI
display('MGI')
[Qtot] = mgi_mitsu(T0P)

% Affichage de la config obtenue et de la matrice de passage correspondante
display('MGD avec Qtot 1 calcul� par MGI')
robot.fkine(Qtot(:,1))
display('MGD avec Qtot 2 calcul� par MGI')
robot.fkine(Qtot(:,2))
% Affichage du robot avec Qtot 1 calcul� par MGI
robot.plot(Qtot(:,1)')
display('appuyer sur une touche pour affichage 2eme solution MGI')
pause
% Affichage du robot Qtot 2 calcul� par MGI
robot.plot(Qtot(:,2)')


%% Calcul d'une trajectoire entre la configuration initiale et celle � 
% atteindre pour l'outil

display('appuyer sur une touche pour jouer la 1ere trajectoire')

% Calcul de la trajectoire cas de la config 1
Q = Qtot(:,1)'; % D�finition de la configuration � atteindre
figure
robot.plot(q) % Affichage du robot � la configuration initiale
hold on
plot3(X,Y,Z,'ro'); % Affichage de la position de la pi�ce dans R0
t = [0:.05:10]; % Vecteur temps n�cessaire pour le calcul de la trajectoire

% A COMPLETER % Calcul de la trajectoire
% jtraj utilise un polynome de degr� 5 avec conditions initiales et finales 
% de vitesse et accel�ration nulles.

robot.plot(Traj_Q) % Affichage de la trajectoire

display('appuyer sur une touche pour jouer la 2nde trajectoire')
pause
% calcul de la trajectoire cas de la config 2

% A COMPLETER


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Commande des deux premiers axes du bras avec Simulink
% Prise en compte de la dynamique du robot (inertie et frottements)
% Imperfections donc correction n�cessaire de la commande des articulations

display('appuyer sur une touche (pour passer � la partie COMMANDE)')
pause
clear q1 q2

% Param�tres Simulink:
r = 1/200; % 1/200   1/30   1   % rapport de r�duction
Km = 0.3; % constante de couple
R = 1; % resistance induit
B = 1/80; % coefficient de frottement
Jeff1 = 0.02; % Inertie efficace articulation 1
Jeff2 = 0.01; % Inertie efficace articulation 2

% Calcul des syst�mes � retour d'Etat:
A1 = [0,1;0,-R*B/Jeff1] 
B1 = [0;Km/Jeff1]
C1 = [1,1]
A2 = [0,1;0,-R*B/Jeff2]
B2 = [0;Km/Jeff2]
C2 = [1,1]
system_1=ss(A1,B1,C1,0);
system_2=ss(A2,B2,C2,0);
rank(ctrb(A1,B1))
rank(ctrb(A2,B2))
rank(obsv(A1,C1))
rank(obsv(A2,C2))
poles_system_1=pole(tf(system_1));
poles_system_2=pole(tf(system_2));

%Les deux syst�mes ne sont pas commentables car le rang de leur matrice de commande est
%1 ce qui est inf�rieur � leur dimension (2)
%Le syst�me 1 admet un p�le valant -0.625, d'o� il est stable et le syst�me
%2 un p�le �gal � -1.25, d'o� il est �galement stable. Mais ces deux syst�mes sont au bord de l'instabilit�
%car ils admettent un deuxi�me pole z�ro

step(system_1)


% Choix des valeurs p�les d�sir�s
p11 = -5+20j
p12 = -5-20j
p21 = -4+20j
p22 = -4-20j
p1=[p11 p12]
p2=[p21 p22]
K1=acker(A1,B1,p1);
Abf1=A1-B1*K1
sysbf1=ss(Abf1,B1,C1,0)
Kp1 = 1/dcgain(sysbf1);
K2=acker(A2,B2,p2);
Abf2=A2-B2*K2
sysbf2=ss(Abf2,B2,C2,0)
Kp2 = 1/dcgain(sysbf2);
q1 = step(sysbf1);
q2 = step(sysbf2);



% Calcul coefficients des correcteurs PD (m�thode au choix):
% A COMPLETER



% D�finition des deux valeurs consignes en fonction de celles trouv�es dans
% le MGI:
qConsigne1 = Q(1); % valeur finale step articulation 1
qConsigne2 = Q(2); % valeur finale step articulation 2
qInitial = 0; % valeur initiale step

% Chargement du mod�le Simulink:
load_system('cmde_robot');
set_param('cmde_robot','SimulationCommand','Start');

display('appuyer sur une touche pour r�cup�rer et jouer la trajectoire')
pause

% R�cup�ration des configs des deux premi�res articulations en sortie
% d'asservissement:
q1_sim = q1; q2_sim = q2; % pour .mdl


% On fixe les valeurs des autres articulations:
q3_simu = zeros(size(q1_sim))-h/2; % vecteur fixe pour articulation 3 (prism.)
q4_simu = zeros(size(q1_sim)); % vecteur de 0 pour articulation 4 (rot.)


% Assemblage des configs des 4 articulations:
% A COMPLETER

% Joue la trajectoire calcul�e via la commande:
robot.plot(q1_sim);
