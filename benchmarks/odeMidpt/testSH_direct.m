function testSH_direct()
  h = 0.0001;
  tn = 40; % stopping time
  y0 = 1; % IC
  tic;
  [t,y] = odeMidpt_testSHfun(tn,h,y0); %  Midpoint integration
  toc;
  fprintf('      t      y_Midpoint\n');
  last = length(t);
  fprintf('%9.4f  %9.6f\n',t(1),y(1))
  fprintf('%9.4f  %9.6f\n',t(last),y(last))
end

function [t,y] = odeMidpt_testSHfun(tn,h,y0)
  t = (0:h:tn)';
  n = length(t);
  y = y0*ones(n,1);
  h2 = h/2;
  for j=2:n
    k1 = testSHfun(t(j-1),y(j-1));
    k2 = testSHfun(t(j-1)+h2,y(j-1)+h2*k1);
    y(j) = y(j-1) + h*k2;
  end
end
