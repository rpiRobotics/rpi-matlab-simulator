

function G = qtdq( q )
    qs = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4); 
    G = 0.5 * [ -qx -qy -qz
                 qs  qz -qy
                -qz  qs  qx
                 qy -qz  qs ];

