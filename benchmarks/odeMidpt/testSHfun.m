function dydt = testSHfun(t, y)
  % Input data taken from demoSteel.m
  %len = 1;     dia = 1e-2;       %  bar length and diameter, meters
  %rho = 7822;  c = 444;          %  density (kg/m^3) and heat capacity (J/kg/K)
  %As = pi*dia*len;               %  surface area of bar, m^2, neglect ends
  %mc = len*0.25*pi*dia^2*rho*c;  %  mc = rho*volume*c
  %emiss = 0.7;                   %  emissivity of bar
  %htc = [15; 100];               %  heat transfer coefficients, W/m^2/C
  %Ta = 21 + 273.15;              %  ambient temperature, K
  %tcool = 70;                    %  begin cooling at tcool seconds
  %tf = 3*tcool;                  %  total simulation time, seconds
  %QV = 3000;                     %  rate of electrical heat generation, W

  mc = 0.25*pi*(1e-4)*3472968;
  As = pi*(1e-2);

  %dydt = fun rhsSteelHeat(t,y,flag,mc,QV,tcool,htc,As,Ta,emiss)
  dydt = rhsSteelHeat(t, y, 0, mc, 3000, 70, [15; 100], As, 284.15, 0.7);
end
