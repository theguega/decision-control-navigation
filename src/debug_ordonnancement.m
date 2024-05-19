
clear;
nav = load("Topologique_VF/nav_topo_vf.mat").nav;

us = user(0,"vert","aubin",35,-1.098835601806642e+02,-1.223507080078125e+02,20.580447592344520/2);

sched = scheduler(nav);
sched.createCar(0,7,-4.867898464202880,-18.820796966552750,1.611867461639450/2);
sched.createCar(1,16,-24.156323432922350,83.218944549560550,19.109999999999980/2);
sched.createCar(2,68,36.468067169189450,1.292723426818850e+02,29.689810552773434/2);


sched.createDemand(82,68,-14.1238908767700,-22.7500019073486,0,24.2548446655273,120.172981262207,0,0,1,2);
sched.schedulDemands();
sched.createDemand(54,115,-21.8339195251465,61.0976829528809,0,15.6448135375977,-125.976043701172,0,0,3,1);
sched.schedulDemands();
sched.createDemand(43,36,80.5898513793945,139.890640258789,0,-119.262542724609,-143.879928588867,0,0,2,1);
sched.schedulDemands();




%[tbl,copied_nav] = us.getClosestVehicle(sched.cars,nav); % récupération des véhicules les plus proches par ordre croissant

% [tbl,copied_nav] = sched.demands(1).getClosestVehicleFromStart(sched.cars,nav); % récupération des véhicules les plus proches par ordre croissant

%showPaths(tbl(1,"path").path,tbl(3,"path").path,copied_nav);


%res = sched.cars(1).getOptimalPath(sched.demands,nav);
%sched.cars(1).assignateNewPath(res.Chemin(1));
% sched.cars(1).showCarPath(nav);
% sched.allocateDemandToCar(0,0);
% sched.allocateDemandToCar(1,0);
% res2 = sched.cars(1).getOptimalPath(sched.demands(1),nav);
% sched.cars(1).assignateNewPath(res2.Chemin(1));
% sched.allocateDemandToCar(2,0);

for i = 1:size(sched.cars,1)
    figure(i);
    sched.cars(i).showCarPath(sched.modified_nav);
end






