

function C = cd_edge_edge( A, B, eps_ee, eps_theta )

    Aid = A.bodyID;
    Bid = B.bodyID; 

    % For now, brute-force
    C = [];
    %edgeList = [];
    
    %% edge-edge A on B
    for e1id = 1:A.num_edges
        
        % t vectors
        tA1 = A.tvecs(e1id,1:3);
        tA2 = A.tvecs(e1id,4:6); 
        if max(abs(tA1+tA2)) < 10^-5, continue; end  % Skip edges with face angle of ~pi
        
        TA = -A.face_norms(A.edges(e1id,3),:) - A.face_norms(A.edges(e1id,4),:); 
        TA = TA/norm(TA);

        % Verts of edge e1
        v1 = A.verts_world(A.edges(e1id,1),:); 
        v2 = A.verts_world(A.edges(e1id,2),:);
        e1 = v2-v1;
        
        % Primary configuration vector
        pcA = cross3(e1, tA1+tA2);  

        %e1ContactCount = 0;
        for e2id = 1:B.num_edges
           
            % t vectors
            tB1 = B.tvecs(e2id,1:3);
            tB2 = B.tvecs(e2id,4:6);
            if max(abs(tB1+tB2)) < 10^-5, continue; end  % Skip edges with face angle of ~pi
            
            TB = -B.face_norms(B.edges(e2id,3),:) - B.face_norms(B.edges(e2id,4),:); 
            TB = TB/norm(TB);

            if dot3(TA, TB) > eps_theta, continue; end
            
            % Verts of edge e2
            v3 = B.verts_world(B.edges(e2id,1),:);    
            v4 = B.verts_world(B.edges(e2id,2),:);
            e2 = v4-v3;
            
            % Calculate unsigned gap distance and nearest points on edges
            [d, ep1, ep2] = segment_segment_distance_3d(v1,v2,v3,v4); % Very expensive to calculate            
            if d > eps_ee, continue; end

            % Primary configuration vector
            pcB = cross3(e2, tB1+tB2); 

            QB = dot3(e2,pcA);
            QA = dot3(e1,pcB);
            
            % Contact normal
            if sign(QA) > 0
                n = cross3(e1,e2)';
            else
                n = cross3(e2,e1)'; 
            end
            n = n / norm(n); 


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% From here, we test which edges to throw out

            %%%%%%%%%%%%%% 
            % For now, ignore planar edges TODO: fix
            if QA == 0 || QB == 0  % Should only check one
              continue; 
            end  

            %%%%%%%%%%%%%% 
            % New applicability function 
            if sign(QA) ~= sign(QB)
              continue;
            end
            
            
            %%%%%%%%%%%%%%
            % Check if e1 "crosses" e2
            if sign(dot3(pcB,v1-ep2)) == sign(dot3(pcB,v2-ep2))
               continue; 
            end
            
            %%%%%%%%%%%%%%
            % Check if both of the nearest edge points are a vertices
            % I believe this is redundant with the "crosses" case above.
            epsV = 10^-5; 
            if norm(ep1-v1) < epsV || norm(ep1-v2) < epsV || ...  % Should be &&
               norm(ep2-v3) < epsV || norm(ep2-v4) < epsV
                continue;
            end
            
            
            %%%%%%%%%%%%%%%%
            % Relaxed applicability
%             dt1 = dot3(n,tA1);
%             dt2 = dot3(n,tA2);
%             dt3 = dot3(n,tB1);
%             dt4 = dot3(n,tB2); 
%             % Check Latombe's app
%             if ~ ((dt1<=0) && (dt2<=0) && (dt3>=0) && (dt4>=0))
%                 % Determine closest tvecs of A and B
%                 disp('check');
%             end
            
            
            %% Passed applicability
            % Here the edge-edge contact has passed all tests of
            % applicability, so is considered valid.  
            psi = dot3(n,ep2-ep1);
            
            if psi > -0.05 % TODO: remove hard-coded epsilon
                C = [ C Contact( Aid, Bid, ep1, n, psi) ];
                %e1ContactCount = e1ContactCount + 1;
                %edgeList = [edgeList; e1id e2id]; 
            end

        end

        
        %% Here, all edges e2 of B have been checked against e1
        % In this case, we may have included too many edge-edge contacts.
        % So we use an edge-edge heuristic to limit the number to 2. 
%         if e1ContactCount > 2
%             sortedC = sort(abs([C(end-e1ContactCount+1:end).psi_n])); 
%             C( abs([C(end-e1ContactCount+1:end).psi_n]) > sortedC(2) ) = []; 
%         end
        
    end
    

end




















