function y = MaxPool(x)
%     
% 2x2 mean pooling
%
%
[xrow, xcol, numFilters] = size(x);

y = zeros(xrow/2, xcol/2, numFilters);
ym = [];
ym1 =[];
for k = 1:numFilters
    for m = 1:2:xrow-1
        for n = 1:2:xcol-1
            max_y = max(max(max(x(m:m+1, n:n+1, k))));
            ym = [ym, max_y(1)];
        end
        ym1 = [ym1; ym];
        ym =[];
    end
    y(:,:,k) = ym1;
    ym1=[];
end
end
 