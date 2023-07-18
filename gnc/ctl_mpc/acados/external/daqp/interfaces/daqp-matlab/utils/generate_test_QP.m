function [x,H,f,A,bupper,blower,sense] = generate_test_QP(n,m,ms,nActive,kappa)
  % * Transform QP to LDP with T = L Q' (where L is diagonal and Q is orthogonal) 
  % to get solution  u =T(x+f) (=> H = T'*T, f = T'*v) 
  % Construct eigenvalues such that cond(H) = kappa 
  eigens = zeros(n,1);   
  eigens(1) = 1; eigens(2) = kappa;
  eigens(3:end) = 1+(kappa-1)*rand(n-2,1);
  % Randomly generate an orthogonal matrix 
  [Q,~] = qr(randn(n));
  T= diag(sqrt(eigens))*Q';
  Tinv = Q*diag(1./sqrt(eigens));
  H = T' * T;
  
  % * Generate solution to min ||u||_2^2 s.t. dlower <= M u <= dupper *
  M = [Tinv(1:ms,:);randn(m-ms,n)]; % First row ms rows corresponds to simple bounds 
  dupper = zeros(m,1);
  dlower= zeros(m,1);
  shuffle_inds= randperm(m);
  nActive_upper = randperm(nActive+1,1)-1;
  nActive_lower= nActive-nActive_upper; 
  ids_active_upper = shuffle_inds(1:nActive_upper);
  ids_active_lower = shuffle_inds(nActive_upper+1:nActive);
  ids_inactive = shuffle_inds(nActive+1:m);

  % * Construct active bounds such that lam>=0 *
  lam = rand(nActive,1); % make lam>=0 by construction
  Ma = [M(ids_active_upper,:);-M(ids_active_lower,:)];
  da=  -Ma*(Ma')*lam;
  dupper(ids_active_upper) = da(1:nActive_upper);
  dlower(ids_active_lower) = -da(nActive_upper+1:nActive);
  u = -Ma'*lam;

  % * Make the inactive constraints feasible *
  bounds_gap = 1; % Scaling factor for distance between bounds
  slack_gap= 1; % Scaling factor for distance between bounds and optimizer 
  dupper(ids_active_lower) = dlower(ids_active_lower)+bounds_gap*(0.01+rand(nActive_lower,1));
  dlower(ids_active_upper) = dupper(ids_active_upper)-bounds_gap*(0.01+rand(nActive_upper,1));
  
  dupper(ids_inactive) = M(ids_inactive,:)*u + slack_gap*(0.01+rand(length(ids_inactive),1)); 
  dlower(ids_inactive) = M(ids_inactive,:)*u - slack_gap*(0.01+rand(length(ids_inactive),1)); 


  % Compute solution x = T\(u-v) (which yields f = (T)'*v)
  v= randn(n,1); f = T'*v;
  x = (T)\(u-v);
  % Transform constraint to x-domain (=> A = M*R, b = d-M*v) 
  A = M(ms+1:end,:)*T;  % Remove simple bounds since they are defined implicitly 
  bupper = dupper-M*v; blower = dlower-M*v;
  sense=zeros(m,1,'int32');
end
