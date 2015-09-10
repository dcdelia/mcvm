function testCamel_direct()
  initial = [0, 0];
  lowerBound = [-10, -10];
  upperBound = [10, 10];
  Mmax = 1200;
  tolerance = 1e-4;
  tic;
  [x0, f0] = sim_anl_camel(initial, lowerBound, upperBound, Mmax, tolerance);
  toc;
  fprintf('x0 is ');
  disp(x0);
  fprintf('f0 is ');
  disp(f0);
end

function [x0,f0]=sim_anl_camel(x0,l,u,Mmax,TolFun)
  if nargin<6
     TolFun=1e-4;
     if nargin<5
         Mmax=100;
     end
  end
  x=x0;fx=camel(x);f0=fx;
  for m=0:Mmax
      T=m/Mmax;
      mu=10^(T*100);
      for k=0:500
          dx=mu_inv(2*rand(size(x))-1,mu).*(u-l);
          x1=x+dx;
          x1=(x1 < l).*l+(l <= x1).*(x1 <= u).*x1+(u < x1).*u;
          fx1=camel(x1);df=fx1-fx;
          if (df < 0 || rand < exp(-T*df/(abs(fx)+eps)/TolFun))==1
              x=x1;fx=fx1;
          end
          if fx1 < f0 ==1
          x0=x1;f0=fx1;
          end
      end
  end
end

function x=mu_inv(y,mu)
  x=(((1+mu).^abs(y)-1)/mu).*sign(y);
end
