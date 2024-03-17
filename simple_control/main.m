% Charger les points depuis le fichier
points = load('points.txt');

% Trouver les limites du plot en fonction des points
min_x = min(points(:, 1)) - 1;
max_x = max(points(:, 1)) + 1;
min_y = min(points(:, 2)) - 1;
max_y = max(points(:, 2)) + 1;

% Paramètres du véhicule et du contrôleur PID
x0 = 0;  % Assurez-vous que ces coordonnées sont correctement initialisées ici
y0 = 0;  % Assurez-vous que ces coordonnées sont correctement initialisées ici
theta0 = 0;
v = 1;
dt = 0.1;
kp = 1;
ki = 0.1;
kd = 0.01;

% Initialiser le véhicule autonome
vehicle = AutonomousVehicle(x0, y0, theta0, v, dt, kp, ki, kd);

% Créer une figure avec un axe fixe et spécifier les limites de l'axe
figure;
ax = gca;
axis(ax, [min_x max_x min_y max_y]);

% Afficher les points en bleu
plot(ax, points(:,1), points(:,2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
hold(ax, 'on');

% Boucle pour déplacer le véhicule vers chaque point
for i = 1:size(points, 1)
    target_x = points(i, 1);
    target_y = points(i, 2);
    disp(['Moving to point (' num2str(target_x) ', ' num2str(target_y) ')']);
    disp(['Vehicle pos : (' num2str(vehicle.x) ', ' num2str(vehicle.y) ')']);
    vehicle.moveToPoint(target_x, target_y, ax);
end