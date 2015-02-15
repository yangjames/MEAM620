function C = min_n_trajectory(path, n, start_constraint, end_constraint, time_stamps)

m = length(path)-1;
C = [];
if length(start_constraint) ~= n || length(end_constraint) ~= n
    return;
end

% calculate distance between pivot points

A = zeros(2*m*n);
X = zeros(2*m*n,size(path,2));

% add position constraints
for i = 1:m
    A((i-1)*2+1:(i-1)*2+2,(i-1)*2*n+1:(i-1)*2*n+2*n) = [time_stamps(i).^((1:2*n)-1);...
                                                        time_stamps(i+1).^((1:2*n)-1)];
    X((i-1)*2+1:(i-1)*2+2,:) = [path(i,:);path(i+1,:)];
end

% add end point constraints
deg = 1:2*n;
for i = 1:n-1
    coeff = deg-i-1;
    coeff(coeff<=0)=0;
    A(m*2+1+2*(i-1),1:2*n) = (factorial(deg-1)./factorial(coeff)).*(time_stamps(1).^((1:2*n)-i-1));
    A(m*2+2+2*(i-1),end-2*n+1:end) = (factorial(deg-1)./factorial(coeff)).*(time_stamps(end).^((1:2*n)-i-1));
    X(m*2+1+2*(i-1),:) = start_constraint(i,:);
    X(m*2+2+2*(i-1),:) = end_constraint(i,:);
end
A(isnan(A) | isinf(A)) = 0;

% add in between constraints
for i = 1:2*(n-1)
    coeff = deg-i-1;
    coeff(coeff<=0)=0;
    for j = 1:m-1
        A((i-1)*(m-1)+j+end-2*(n-1),(j-1)*2*n+1:(j-1)*2*n+4*n)=...
            [(factorial(deg-1)./factorial(coeff)).*(time_stamps(j+1).^((1:2*n)-i-1)) -(factorial(deg-1)./factorial(coeff)).*(time_stamps(j+1).^((1:2*n)-i-1))];
    end
end
C=A;
%C = A\X;