function testSH()
  fun = 'testSHfun';
  h = 0.0001;
  tn = 20; % stopping time
  y0 = 1; % IC
  tic;
  [t,y] = odeRK4(fun,tn,h,y0); %  Fourth order Runge-Kutta method
  toc;
  fprintf('      t      y_RK4\n');
  last = length(t);
  fprintf('%9.4f  %9.6f\n',t(1),y(1))
  fprintf('%9.4f  %9.6f\n',t(last),y(last))
end
