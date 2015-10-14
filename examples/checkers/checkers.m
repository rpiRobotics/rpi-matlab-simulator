

function [ game ] = checkers( )

    game = Simulator();
    
    %% Board
    Board = [];
    for row = 1:8
       for col = 1:8
          sqr = mesh_rectangularBlock(1,1,.2);
          if mod(row+col,2)
              sqr.color = [1 0 0];
          else
              sqr.color = [0 0 0];
          end
          sqr.u = [row; col; -.1];
          sqr.facealpha = 1;
          sqr.edgealpha = 0; 
          sqr.dynamic = false; 
          Board = [Board sqr];
       end
    end
    game = sim_addBody(game,Board); 

    %% Pieces
    Pieces = [];
    lightColor = [.8 .7 .2];
    darkColor = [.5 .3 0];

    % Light pieces
    COLs = [1 3 5 7 2 4 6 8 1 3 5 7];
    c = 1;
    for row = 1:3
        for col = 1:4
            piece = mesh_cylinder(20,1,.4,.2);
            piece.u = [COLs(c) row .1];
            piece.color = lightColor;
            piece.facealpha = 1;
            piece.edgealpha = 0.1; 
            piece.dynamic = false; 
            Pieces = [Pieces piece];
            c = c+1; 
        end
    end
    
    % Dark pieces
    COLs = [2 4 6 8 1 3 5 7 2 4 6 8];
    c = 1;
    for row = 8:-1:6
        for col = 1:4
            piece = mesh_cylinder(20,1,.4,.2);
            piece.u = [COLs(c) row .1];
            piece.color = darkColor;
            piece.facealpha = 1;
            piece.edgealpha = 0.2; 
            piece.dynamic = false; 
            Pieces = [Pieces piece];
            c = c+1; 
        end
    end
    
    game = sim_addBody(game,Pieces); 
    
    %game.MAX_STEP = 5;
    game.userFunction = @playCheckers; 
    game = sim_run( game );
    
    
end









