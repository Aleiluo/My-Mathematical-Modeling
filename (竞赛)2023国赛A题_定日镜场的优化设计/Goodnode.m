% Assume to find M points distributed evenly among the decision space,[0,1]^N.
% The Good-Point Array (or the Good Nodes Set, the Good Point Set) can provide 
% a set of point coordinates which distribute in a more even and more stable 
% way than the uniformly random points set.
% This algorithm provides the Good-Point Array with while inputting M and N.
% where M is the number of points; N is the dimensionality of each point;
% The output is an M*N array in which each row represent a point coordinate
% in the [0,1]^N  decision space.
% Reference: 
% [1]Chixin Xiao, Zixing Cai, and Yong Wang, “A good nodes set 
% evolution strategy for constrained optimization,” in 2007 IEEE Congress on 
% Evolutionary Computation, 2007, pp. 943–950.
% Email: chixinxiao@gmail.com , Chixin Xiao, UoW ,AU , 23 May 2019

function [GD] = Goodnode(M,N)
% M is the number of points; N is the dimension 
if (nargin==0)
    M=100;
    N=2;
end
%%
tmp1 = [1:M]'*ones(1,N);
Ind = [1:N];
prime1 = primes(100*N);
[p,q]=find(prime1>=(2*N+3));
tmp2 = (2*pi.*Ind)/prime1(1,q(1));
tmp2 = 2*cos(tmp2);
tmp2 = ones(M,1)*tmp2;
GD = tmp1.*tmp2;
GD = mod(GD,1);
%% For debuging
% plot(GD(:,1),GD(:,2),'*');
end

