% PCB dimensions
PCB_Thi = 1.6e-3;
PCB_Len = 20e-3;
PCB_Wid = 22e-3;
PCB_Material = 'FR4';
PCB_Epsilon_R = 4.4;

% set up model
PCB_d = dielectric(PCB_Material);
PCB_d.EpsilonR = PCB_Epsilon_R;
PCB_d.Thickness = PCB_Thi;

GND_Plane = antenna.Rectangle('Length',PCB_Len,'width',PCB_Wid);    % ground plane

ANT_Plane = antenna.Rectangle('Length',5e-3,'width',5e-3,'Center',[0,0]);   % ant plane

% set up stack
p = pcbStack;
p.Name = 'PCB Antenna';
p.BoardShape = GND_Plane;
p.BoardThickness = PCB_Thi;
p.Layers = {ANT_Plane,PCB_d,GND_Plane};
p.FeedLocations = [0,0,1,3];        % x, y, starting layer, ending layer
p.Layers = {ANT_Plane,PCB_d,GND_Plane};

% drawing results
figure(1);
show(p);

figure(2);
pattern(p,868e6);       % show radiation pattern at 868MHz

figure(3);
impedance(p,820e6:5e6:920e6);     % show mpedence graph from 820MHz to 920Mhz in steps of 5MHz

% freq_plot = linspace(820e6,920e6,50);   % 50 points from 820MHz to 920Mhz
% s = sparameters(p,freq_plot,50);        % plot S11 at frequencies
% figure(4);
% rfplot(s);


