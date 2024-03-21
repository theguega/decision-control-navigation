%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Université de Technologie de Compiègne (UTC)                       %
%% SY28 :: Département Génie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part I : Planning and control of mobile robots                     %
%% Théorème de stabilité de Lyapunov et méthode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dernière modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function DessinUnicyle(Xreel,Yreel,theta, Info)

global d r

% --- Position du centre de l'essieu arrière
xe = Xreel;
ye = Yreel;

%===================
% Essieu Arrière  ||
%===================
% --- Centre de la roue arrière droite
xArD = xe + d/2*sin(theta);
yArD = ye - d/2*cos(theta);
% 
% % --- Centre de la roue arrière gauche
xArG = xe - d/2*sin(theta);
yArG = ye + d/2*cos(theta);

%========================
% Roue Arrière Gauche  ||
%========================
xArGDev = xArG + r*cos(theta);
yArGDev = yArG + r*sin(theta);

xArGArr = xArG - r*cos(theta);
yArGArr = yArG - r*sin(theta);
% 
% %========================
% % Roue Arrière Droite  ||
% %========================
xArDDev = xArD + r*cos(theta);
yArDDev = yArD + r*sin(theta);

xArDArr = xArD - r*cos(theta);
yArDArr = yArD - r*sin(theta);

%%Position du centre du point situé devant le robot 
xA = xe + (0.5)*cos(theta);
yA = ye + (0.5)*sin(theta);

%===========================
% Tracé du Robot Mobile  ||
%===========================
figure(1)
hold on
plot(xe,ye,'r+');              % Tracé du centre de l'essieu arrière
grid on
%line([xe xA],[ye yA])         % Tracé de l'empattement -- relie (x,y) à (xA,yA) 
%plot(xA, yA, 'b+')            % Tracé du devant du robot   
line([xArG xArD],[yArG yArD])                           % Tracé de l'essieu arrière -- relie (xArG,yArG) à (xArD,yArD)
line([xArGDev xArGArr],[yArGDev yArGArr],'Color','r')   % Tracé de la roue arrière gauche -- relie (xArGDev,yArGDev) à (xArGArr,yArGArr)
line([xArDDev xArDArr],[yArDDev yArDArr],'Color','r')   % Tracé de la roue arrière droite -- relie (xArDDev,yArDDev) à (xArDArr,yArDArr)

for i=0:0.2:(2*pi)
    plot(xe+(d/2)*cos(i),ye+(d/2)*sin(i),'-');
end

%Dessin de l'info souhaitée
text(xe, ye+0.5, num2str(Info))

