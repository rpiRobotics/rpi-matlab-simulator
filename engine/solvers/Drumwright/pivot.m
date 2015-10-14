% pivoting algorithm for determining limited number of contact forces
% (for frictionless contacts)
function [z, pivots, errcode] = pivot(M, N, v, PSI)

  EPS = 1e-4;

  % get number of generalized coordinates
  m = size(M,1);

  % setup the basic variable and nonbasic variable indices
  n = size(N,1);
  nbas = [];
  bas = 1:n;

  % compute N*v and minimum indices
  Nv = N*v + PSI;
  [minx, mini] = min(Nv);

  % if N*v >= 0, use z=0
  if (minx >= -EPS)
    z = zeros(n,1);
    return;
  end

  % move that index to the nbasic set
%  nbas = mini(ceil(rand(1)*length(mini)));
  nbas = mini;
  basi = find(bas == mini);
  bas(basi) = [];
  % fprintf(1, 'Inserted index %d into non-basic set\n', mini);

  % setup number of pivots
  pivots = 0;

  while (pivots < 100)

    % update number of pivots
    pivots = pivots + 1;

    if (pivots > 2^n)
      nbas
      bas
    end

    % verify that number of indices in non-basic set has not exceeded m
    if (length(nbas) > m)
      fprintf(1, 'PPM: invariant failed\n');
      z = [];
      errcode = -1;
      return; 
    end

    % solve for nonbasic z
    Asub = N(nbas,:) * (M \ N(nbas, :)');
    bsub = (N(nbas,:)*v) + PSI(nbas);
    za = Asub \ -bsub;

    % recompute N*v and minimum indices
    vplus = M \ (N(nbas,:)'*za) + v;
    Nvplus = N*vplus + PSI;
    [minx, mini] = min(Nvplus);

    % if Nvplus >= 0, check whether any component of z < 0
    if (minx >= -EPS)
      minz = min(za);
      if (minz < -EPS)
        % search for multiple minimum indices
        mini = find(abs(za - minz) < EPS);
        mini = mini(ceil(rand()*length(mini))); 

        % move index to basic set
        bas = [bas nbas(mini)];
        nbas(mini) = [];
        if (pivots > 2^n) 
          fprintf(1, 'Inserted index %d into non-basic set\n', mini);
        end
        continue;
      else
        % we're done!
        z = zeros(1,n);
        z(nbas) = za;
        z = z';
        errcode = 0;
        return;
      end
    else
      % get minimum component of Nvplus and move it to nonbasic set
      nbas = [nbas mini];
      basi = find(bas == mini);
      bas(basi) = []; 
      if (pivots > 2^n) 
        fprintf(1, 'Inserted index %d into non-basic set\n', mini);
      end

      % look whether any components of z needs to move to basic set
      minz = min(za);
      if (minz < -EPS)
        % search for multiple minimum indices
        mini = find(abs(za - minz) < EPS);
        mini = mini(ceil(rand()*length(mini))); 

        % move index to basic set
        bas = [bas nbas(mini)];
        nbas(mini) = []; 
        if (pivots > 2^n) 
          fprintf(1, 'Inserted index %d into basic set\n', mini);
        end
      end      
    end
  end

  % made it here? maximum number of pivots exceeded
  fprintf(1,'Maximum number of pivots exceeded!\n');
  z = [];
  errcode = -1;


