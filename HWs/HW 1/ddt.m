function dx=ddt(x,incr)
%
%	THIS FUNCTION APPROXIMATES THE DERIVATIVE OF THE
%	INPUT x BY THE FITTING A PARABOLA TO FIVE POINTS 
%	AND USING ITS ANALYTICAL DERIVATIVE.  THE EQUATION 
% 	FOR THIS APPROXIMATION IS:
%
%	    DX(N)=[-2X(N-2) - X(N-1) + X(N+1) +2X(N+2)]/(10*incr)
%
%	EXCEPT FOR THE FIRST TWO AND LAST TWO POINTS WHICH USE:
%
%	    DX(1)=   [-21X(1) + 13X(2) + 17X(3) - 9X(4)]/(20*INCR)		
%	    DX(2)=   [-11X(1) + 3X(2) + 7X(3) + X(4)]/(20*INCR)		
%	    DX(M-1)= [11X(M) - 3X(M-1) - 7X(M-2) - X(M-3)]/(20*INCR)		
%	    DX(M)=   [21X(M) - 13X(M-1) - 17X(M-2) + 9X(4M-3)]/(20*INCR)	
%
%	SEE SCHAUM'S OUTLINE: NUMERICAL ANALYSIS, PP. 245-246 FOR DETAILS.	
%
%	USAGE:	dx=ddt(x,incr)
%
%	IF THE INCREMENT IS NOT SUPPLIED IT IS ASSUMED TO BE 1
%
% RFK Jan 1992
% EJP Aug 1998 - allowed columns of data to be processed
if (nargin == 1)
	incr = 1;
end

[xr,xc]=size(x);
numinpts=min(xr,xc);
if numinpts>1 %Process each column
   for i=1:numinpts dx(:,i)=ddt(x(:,i),incr);end
   return
end

dx=filter22([-2 -1 0 1 2],x,2);
dx=-dx/(10.*incr);
m=length(x);
dx(1) = (-21.*x(1)+13.*x(2)+17.*x(3)-9.*x(4))/(20.*incr);
dx(2)=  (-11.*x(1) + 3.*x(2) + 7.*x(3) + x(4))/(20.*incr);
dx(m) = (21.*x(m)-13.*x(m-1)-17.*x(m-2)+9.*x(m-3))/(20.*incr);
dx(m-1)=  (11.*x(m) - 3.*x(m-1) - 7.*x(m-2) - x(m-3))/(20.*incr);
[dxr,dxc]=size(dx);
if (dxr ~= xr)
	dx=dx';
end

