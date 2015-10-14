

function DATA = randomPositionAnalysis()

    N = 100000; 
    
    % Data(iteration) = [penetration applicability]
    DATA = zeros(N,2); 

    A = mesh_tetrahedron;
        A.color = [.4 .4 .4]; 
        A = scale_mesh( A, .8 ); 
        ra_local = A.verts_local(1,:) + 0.5*(A.verts_local(4,:)-A.verts_local(1,:));
        ea = 4;
    B = mesh_tetrahedron;
        rb_local = B.verts_local(1,:) + 0.5*(B.verts_local(4,:)-B.verts_local(1,:));
        eb = 4; 
        
    % Plot for debugging
    A = body_draw_init(A);
    hold on;
    B = body_draw_init(B);
    plot3(0,0,0,'r*'); 
    axis equal;  
    view(3); 
    rotate3d; 
    grid on; 
    
    for i=1:N
       % Randomly orient two bodies, and position such that ra and rb coincide
       %A.quat = qt(rand(3,1), rand*2*pi);
       B.quat = qt(rand(3,1), rand*2*pi);
       %B.quat = qtmultiply( qt([0 0 1],pi/4), qt([-1 0 1],pi/2)); 
       ra = qtrotate( A.quat, ra_local' );
       rb = qtrotate( B.quat, rb_local' );
       A.u = -ra*(1.00001);
       B.u = -rb;
       
       % Redraw
       A = body_updateMesh(A);
       B = body_updateMesh(B);
       %body_draw(A);
       %body_draw(B); 
       % Draw edges
       %highlightEdge(A,ea);
       %highlightEdge(B,eb); 
       
       % Test for penetration
       penetration = false; 
       for fa=1:A.num_faces
          Fa = A.faces(fa,:);
          fa0 = A.verts_world(Fa(1),:);
          fa1 = A.verts_world(Fa(2),:);
          fa2 = A.verts_world(Fa(3),:);
          for fb=1:B.num_faces
            Fb = B.faces(fb,:);
            fb0 = B.verts_world(Fb(1),:);
            fb1 = B.verts_world(Fb(2),:);
            fb2 = B.verts_world(Fb(3),:);
            
            penetration = triTriContact([fa0; fa1; fa2], [fb0; fb1; fb2]);
            if penetration, break; end
          end
          if penetration, break; end
       end
       
       DATA(i,1) = penetration;
       DATA(i,2) = applicability_ee(A,B,ea,eb); 
       
       if penetration
          title('Penetration');
       else
          title('Not penetrating'); 
       end
       
       %drawnow; 
       %pause(.1); 
    end
    
    data_penetration = DATA(DATA(:,1)==1,2);
    data_no_penetration = DATA(DATA(:,1)==0,2);
    figure;
    hist(data_penetration,15);
    title('Applicability when in PENETRATION');
    figure;
    hist(data_no_penetration,15);
    title('Applicability when in NO penetration');
    

end












