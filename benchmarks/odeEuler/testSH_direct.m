function testSH_direct()
  h = 0.0001;
  tn = 80; % stopping time
  y0 = 1; % IC
  tic;
  [t,y] = odeEuler_testSHfun(tn,h,y0); %  Euler integration
  toc;
  fprintf('      t      y_Euler\n');
  last = length(t);
  fprintf('%9.4f  %9.6f\n',t(1),y(1))
  fprintf('%9.4f  %9.6f\n',t(last),y(last))
end

function [t,y] = odeEuler_testSHfun(tn,h,y0)
  t = (0:h:tn)';
  n = length(t);
  y = y0*ones(n,1);
  for j=2:n
    y(j) = y(j-1) + h*testSHfun(t(j-1),y(j-1));
  end
end
