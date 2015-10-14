

function numContacts = EE_drawValid( B1, B2 )

epsTheta = 0.5;  

numContacts = 0;

re1 = 0;
re2 = 0;
re3 = 0;
re4 = 0;
re5 = 0;
re6 = 0;

for e1id = 1:B1.num_edges
  for e2id = 1:B2.num_edges

      % The edges
      E1 = B1.edges(e1id,:);
      E2 = B2.edges(e2id,:); 
      
      % The vertices 
      v1 = B1.verts_world(E1(1),:); 
      v2 = B1.verts_world(E1(2),:);
      v3 = B2.verts_world(E2(1),:); 
      v4 = B2.verts_world(E2(2),:);
      
      % Calculate unsigned gap distance and nearest points on edges
      [d, ep1, ep2] = segment_segment_distance_3d(v1, v2, v3, v4);
      
      %plot3([ep1(1) ep2(1)],[ep1(2) ep2(2)],[ep1(3) ep2(3)],'r');
          
      % Vectors (un-normalized) representing the edges
      e1 = v2-v1;
      e2 = v4-v3;
      
      % t vectors
      tA1 = B1.tvecs(e1id,1:3);
      tA2 = B1.tvecs(e1id,4:6);
      tB1 = B2.tvecs(e2id,1:3);
      tB2 = B2.tvecs(e2id,4:6);

      % Primary configuration vectors
      pcA = cross(e1, tA1+tA2);  % 
      pcB = cross(e2, tB1+tB2); 

      QA = dot(e1,pcB);
      QB = dot(e2,pcA);

      BA = B1.u - B2.u;
      BA = BA / norm(BA); 

      % Calculate some normals
      %nAB = EE_calculateNormal(e1,e2,QA);
      %nBA = EE_calculateNormal(e2,e1,QB);
      

      valid = true;
      
      %%%%%%%%%%%%%% 
      if QA == 0 || QB == 0  % Should only check one
          disp(['1. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
          %continue; 
          valid = false;
          re1 = re1+1; 
      end  
      %nAB = EE_calculateNormal(e1,e2,QA);
      %nBA = EE_calculateNormal(e2,e1,QB);
      
      %%%%%%%%%%%%%% 
      % New applicability function 
      if sign(QA) ~= sign(QB)
          disp(['2. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
          %continue;
          valid = false;
          re2 = re2+1;
      end
      
      
      
      %%%%%%%%%%%%%%
      % Check if both of the nearest edge points are a vertices
%         epsV = 10^-5; 
%         if norm(ep1-v1) < epsV || norm(ep1-v2) < epsV || ...  % Should be &&
%            norm(ep2-v3) < epsV || norm(ep2-v4) < epsV
%             valid = false;
%             re3 = re3+1; 
%         end
      
      
      %%%%%%%%%%%%%% 
      % Checks if edge is on opposite side of body from contact
%       if QA ~= 0 && QB ~= 0 && sign(QA) == sign(QB)
%           n = EE_calculateNormal(e1,e2,QA); 
%           pn = ep2-ep1;
%           psi = dot(n,pn); 
%           if psi < 0
%              disp('negative psi'); 
%           end
%           disp('negative psi'); 
%           if dot( ep1-B1.u , n ) < 0 || dot( ep2-B2.u , -n ) < 0
%               disp(['4. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
%               %continue;
%               valid = false;
%               re4 = re4+1;  
%           end
%       end
      

      % This is throwing out contacts with small negative gaps
      % Also, what if ep2-ep1 is zero...
%       if QA ~= 0 && QB ~= 0 && sign(QA) == sign(QB) && valid
%           n = EE_calculateNormal(e1,e2,QA); 
%           pn = ep2-ep1;
%           psi = dot(n,pn); 
%           if psi < 0
%               pn = -pn;
%           end
%           if norm(pn)>0 
%               if dot(pn, B2.u-B1.u) < 0 || ...
%                  dot(ep1-B1.u, pn) < 0 || dot(ep2-B2.u, -pn) < 0
%                   disp(['4. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
%                   %continue;
%                   valid = false;
%                   re4 = re4+1; 
%               end
%           elseif dot( ep1-B1.u , n ) < 0 || dot( ep2-B2.u , -n ) < 0
%               disp(['4. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
%               %continue;
%               valid = false;
%               re4 = re4+1; 
%           end
%       end
          
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      if QA ~= 0 && QB ~= 0 && sign(QA) == sign(QB) && valid
          n = EE_calculateNormal(e1,e2,QA); 
          pn = ep2-ep1;
          psi = dot(n,pn); 
          if psi < 0
              pn = -pn;
          end
          pn = pn/norm(pn);

          disp('t dot products:');
          ka1 = sign( dot(tA1, n) );   % TODO: use pn instead of n if one
          ka2 = sign( dot(tA2, n) );   % of the ep is a vertex 
          kb1 = sign( dot(tB1, n) );    
          kb2 = sign( dot(tB2, n) );  

          if ka1 == ka2 && ka2 == kb1 && kb1 == kb2  
             disp(['5. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
              %continue;
              valid = false;
              re5 = re5+1; 
          end
      end
      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      if QA ~= 0 && QB ~= 0 && sign(QA) == sign(QB) && valid
          b1ep1 = ep1-B1.u';  b1ep1 = b1ep1/norm(b1ep1);
          b1ep2 = ep2-B1.u';  b1ep2 = b1ep2/norm(b1ep2); 
          b2ep2 = ep2-B2.u';  b2ep2 = b2ep2/norm(b2ep2);
          b2ep1 = ep1-B2.u';  b2ep1 = b2ep1/norm(b2ep1);
          if dot(b1ep1, b1ep2) < epsTheta || dot(b2ep2,b2ep1) < epsTheta
             disp(['6. Dropping ' num2str(e1id) ' and ' num2str(e2id) ]);
              %continue;
              valid = false;
              re6 = re6+1; 
          end
      end
      
      
      
      % Throw out or keep
      if ~valid
          continue;
      end

      % Draw from mid edge to mid edge
      m1 = v1 + 0.5*e1;
      m2 = v3 + 0.5*e2; 
      plot3([m1(1) m2(1)],[m1(2) m2(2)],[m1(3) m2(3)],'r','linewidth',2);

      numContacts = numContacts + 1;
  end
end


re1
re2 
re3
re4
re5
re6


