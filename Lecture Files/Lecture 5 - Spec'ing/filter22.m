function y = filter22(fil,x,numsides)
%
%	THIS FUNCTION PERFORMS 2-SIDED AS WELL AS ONE SIDED
% 	FILTERING.  NOTE THAT FOR A ONE-SIDED FILTER, THE 
%	FIRST length(fil) POINTS ARE GARBAGE, AND FOR A TWO
%	SIDED FILTER, THE FIRST AND LAST length(fil)/2 
%	POINTS ARE USELESS.
%
%	USAGE	: y = filter22(fil,x,numsides)
%
% EJP Jan 1991
%
[ri,ci]= size(x);
if (ci > 1)
     	x = x';
end
numpts = length(x);
halflen = ceil(length(fil)/2);
if numsides == 2
	x = [x ; zeros(size(1:halflen))'];
	y = filter(fil,1,x);
	y = y(halflen:numpts + halflen - 1);
else 
	y=filter(fil,1,x);
end
if (ci > 1)
     	y = y';
end
return
