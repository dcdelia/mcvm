function testSHdirect()
  h = 0.0001;
  tn = 20; % stopping time
  y0 = 1; % IC
  tic;
  [t,y] = odeRK4_testSHfun(tn,h,y0); %  Fourth order Runge-Kutta method
  toc;
  fprintf('      t      y_RK4\n');
  last = length(t);
  fprintf('%9.4f  %9.6f\n',t(1),y(1))
  fprintf('%9.4f  %9.6f\n',t(last),y(last))
end

function [t,y] = odeRK4_testSHfun(tn,h,y0)
  t = (0:h:tn)';                   %  Column vector of elements with spacing h
  n = length(t);                   %  Number of elements in the t vector
  y = y0*ones(n,1);                %  Preallocate y for speed
  h2 = h/2;  h3 = h/3;  h6 = h/6;  %  Avoid repeated evaluation of constants
  for j=2:n
    k1 = testSHfun(t(j-1),    y(j-1)        );
    k2 = testSHfun(t(j-1)+h2, y(j-1)+h2*k1  );
    k3 = testSHfun(t(j-1)+h2, y(j-1)+h2*k2  );
    k4 = testSHfun(t(j-1)+h,  y(j-1)+h*k3   );
    y(j) = y(j-1) + h6*(k1+k4) + h3*(k2+k3);
  end
end
