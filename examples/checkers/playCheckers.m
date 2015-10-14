

function game = playCheckers( game )

    if game.step == 1
       %camproj('perspective');  
       view(0,45); 
       
       % Init some vars
       game.userData.prevPiece = 65;
       game.userData.prevPieceColor = game.bodies(65).color;
       game.userData.lastSquare = 1; 
       game.userData.lastSquareColor = game.bodies(1).color; 
       
       % Init the board 
       game.userData.BOARD = int8( zeros(8,8) );
       game.userData.BOARD(1,1) = 1;
       game.userData.BOARD(3,1) = 1;
       game.userData.BOARD(5,1) = 1;
       game.userData.BOARD(7,1) = 1;
       game.userData.BOARD(2,2) = 1;
       game.userData.BOARD(4,2) = 1;
       game.userData.BOARD(6,2) = 1;
       game.userData.BOARD(8,2) = 1;
       game.userData.BOARD(1,3) = 1;
       game.userData.BOARD(3,3) = 1;
       game.userData.BOARD(5,3) = 1;
       game.userData.BOARD(7,3) = 1;
       
       game.userData.BOARD(2,8) = 2;
       game.userData.BOARD(4,8) = 2;
       game.userData.BOARD(6,8) = 2;
       game.userData.BOARD(8,8) = 2;
       game.userData.BOARD(1,7) = 2;
       game.userData.BOARD(3,7) = 2;
       game.userData.BOARD(5,7) = 2;
       game.userData.BOARD(7,7) = 2;
       game.userData.BOARD(2,6) = 2;
       game.userData.BOARD(4,6) = 2;
       game.userData.BOARD(6,6) = 2;
       game.userData.BOARD(8,6) = 2;
    else
        
        pause(1); 
        
        %% Get the piece
        % Clear previous clicks
        game.bodies(game.userData.prevPiece).color = game.userData.prevPieceColor; 
        game.bodies(game.userData.lastSquare).color = game.userData.lastSquareColor; 
        body_draw(game.bodies(game.userData.prevPiece));
        body_draw(game.bodies(game.userData.lastSquare));
        
        
        % Get user data
        [x1,y1,button] = ginput(1);  
        
        % Find body closest to click
        pieceClicked = 65;
        minDist = inf;
        for b=65:length(game.bodies)
            pos = game.bodies(b).u;
            sqDist = (x1-pos(1))^2 + (y1-pos(2))^2;
            if sqDist < minDist
                pieceClicked = b;
                minDist = sqDist; 
            end
        end
        
        % Handle the click
        game.userData.prevPiece = pieceClicked; 
        game.userData.prevPieceColor = game.bodies(pieceClicked).color; 
        
        game.bodies(pieceClicked).color = [0 0 1];
        body_draw(game.bodies(pieceClicked)); 
        
        %% Get the square
         % Get user data
        [x2,y2,button] = ginput(1);  
        
        % Find body closest to click
        squareClicked = 1;
        minDist = inf;
        for b=1:64
            pos = game.bodies(b).u;
            sqDist = (x2-pos(1))^2 + (y2-pos(2))^2;
            if sqDist < minDist
                squareClicked = b;
                minDist = sqDist; 
            end
        end
        
        % Handle the click
        game.userData.lastSquare = squareClicked;
        game.userData.lastSquareColor = game.bodies(squareClicked).color; 
        
        game.bodies(squareClicked).color = [0 1 0];
        body_draw(game.bodies(squareClicked)); 
        
        
        %% Move the piece to the square that was clicked
        % Check legality of move
        x1 = round(x1); y1 = round(y1);
        x2 = round(x2); y2 = round(y2); 
        if isLegalMove( game.userData.BOARD , x1,y1, x2,y2)
            game.bodies(pieceClicked).u(1:2) = game.bodies(squareClicked).u(1:2); 
        
            % Update board
            game.userData.BOARD(x2,y2) = game.userData.BOARD(x1,y1); 
            game.userData.BOARD(x1,y1) = 0;
            % Remove jumped piece
            if abs(y2-y1) == 2
                for b=65:length(game.bodies)
                   if game.bodies(b).u(1) == mean([x1,x2]) && game.bodies(b).u(2) == mean([y1,y2])
                       set(game.bodies(b).graphicsHandle, 'visible', 'off'); 
                       disp('Jump!'); 
                       break;
                   end
                end
                game.userData.BOARD(mean([x1,x2]),mean([y1,y2])) = 0; 
            end
        else
           disp('Illegal move'); 
        end
        
        
    end

end

