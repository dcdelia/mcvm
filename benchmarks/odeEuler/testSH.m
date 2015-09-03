function testSH()
  fun = 'testSHfun';
  h = 0.0001;
  tn = 80; % stopping time
  y0 = 1; % IC
  tic;
  [t,y] = odeEuler(fun,tn,h,y0); %  Euler integration
  toc;
  fprintf('      t      y_Euler\n');
  last = length(t);
  fprintf('%9.4f  %9.6f\n',t(1),y(1))
  fprintf('%9.4f  %9.6f\n',t(last),y(last))
end
