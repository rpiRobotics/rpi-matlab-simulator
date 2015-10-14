
% Given a board position and move from square s1 = x1,y1 to square 
% s2 = x2,y2, returns true if the move is legal.  
% 
function isLegal = isLegalMove( board, x1,y1, x2,y2 )

    % Clip range 
    if any( [x1<1 x1>8 y1<1 y1>8 x2<1 x2>8 y2<1 y2>8]) 
        isLegal = false; return; 
    end

    % Make sure both x1,y1 and x2,y2 are dark squares
    if mod(x1,2) ~= mod(y1,2) || mod(x2,2) ~= mod(y2,2)
        isLegal = false; return;
    end
    
    % Make sure a piece is at x1,y1
    if ~board(x1,y1), isLegal = false; return; end
    
    % Make sure no piece is at x2,y2
    if board(x2,y2), isLegal = false; return; end
    
    % If light piece
    if board(x1,y1) == 1
     
        % If not a king
        if board(x1,y1) > 0
           % If not jumping 
           if y2 == y1+1
               % Moving left or right
               if abs(x2-x1) == 1
                   isLegal = true; return;
               else
                   isLegal = false; return;
               end

           % If jumping forward
           elseif y2 == y1+2  
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end

           % Else if jumping backward
           elseif y2 == y1-2
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end
           else
               isLegal = false; return;
           end

        % If a king (nearly identical to above)
        else
            % If not jumping 
           if abs(y2-y1) == 1    % <--- this is the only difference
               % Moving left or right
               if abs(x2-x1) == 1
                   isLegal = true; return;
               else
                   isLegal = false; return;
               end

           % If jumping forward
           elseif y2 == y1+2  
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end

           % Else if jumping backward
           elseif y2 == y1-2
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end
           else
               isLegal = false; return;
           end
        end

    % Else if dark piece
    else
        % If not a king
        if board(x1,y1) > 0
           % If not jumping 
           if y2 == y1-1
               % Moving left or right
               if abs(x2-x1) == 1
                   isLegal = true; return;
               else
                   isLegal = false; return;
               end

           % If jumping forward
           elseif y2 == y1-2  
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end

           % Else if jumping backward
           elseif y2 == y1+2
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end
           else
               isLegal = false; return;
           end

        % If a king (nearly identical to above)
        else
            % If not jumping 
           if abs(y2-y1) == 1    % <--- this is the only difference
               % Moving left or right
               if abs(x2-x1) == 1
                   isLegal = true; return;
               else
                   isLegal = false; return;
               end

           % If jumping forward
           elseif y2 == y1-2  
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end

           % Else if jumping backward
           elseif y2 == y1+2
               if abs(x2-x1) == 2
                   isLegal = true; return;
               else
                   isLegal = false; return; 
               end
           else
               isLegal = false; return;
           end
        end
    end
        
end









