function test1()
  fun = 'camel';
  initial = [0, 0];
  lowerBound = [-10, -10];
  upperBound = [10, 10];
  %Mmax = 400;
  Mmax = 1200;
  tolerance = 1e-4;
  tic;
  [x0, f0] = sim_anl(fun, initial, lowerBound, upperBound, Mmax, tolerance);
  toc;
  fprintf('x0 is ');
  disp(x0);
  fprintf('f0 is ');
  disp(f0);
end
