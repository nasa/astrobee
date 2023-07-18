function [x,f,A,bupper,blower,sense] = generate_test_LP(n,m,ms)
  A = [eye(ms,n);randn(m-ms,n)];
  bupper = zeros(m,1);
  blower = zeros(m,1);
  shuffle_inds= randperm(m);
  nActive_upper = randperm(n+1,1)-1;
  nActive_lower= n-nActive_upper; 
  ids_active_upper = shuffle_inds(1:nActive_upper);
  ids_active_lower = shuffle_inds(nActive_upper+1:n);
  ids_inactive = shuffle_inds(n+1:m);

  lam = rand(n,1); % make dual feasible (lam >= 0
  x = randn(n,1);

  Aa = [A(ids_active_upper,:);-A(ids_active_lower,:)];
  f = -Aa'*lam;

  ba = Aa*x;
  bupper(ids_active_upper) = ba(1:nActive_upper);
  blower(ids_active_lower) = -ba(nActive_upper+1:n);

  
  % * Make the inactive constraints feasible *
  bounds_gap = 1; % Scaling factor for distance between bounds
  slack_gap= 1; % Scaling factor for distance between bounds and optimizer 
  bupper(ids_active_lower) = blower(ids_active_lower)+bounds_gap*(0.01+rand(nActive_lower,1));
  blower(ids_active_upper) = bupper(ids_active_upper)-bounds_gap*(0.01+rand(nActive_upper,1));
  
  bupper(ids_inactive) = A(ids_inactive,:)*x + slack_gap*(0.01+rand(length(ids_inactive),1)); 
  blower(ids_inactive) = A(ids_inactive,:)*x - slack_gap*(0.01+rand(length(ids_inactive),1)); 
  A = A(ms+1:end,:); % simple bounds are implicitly define so remove them from A.
  sense = zeros(m,1,'int32');
end
