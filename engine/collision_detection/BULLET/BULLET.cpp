
// This is a test mex function that is based on a modified version of 
// BULLET's "helloWorld" example:
// http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

#include "mex.h"
#include "btBulletCollisionCommon.h"

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
    double *verts, *vertOffset, *pos, *quat, *bID, *nb, *nv;
    verts = mxGetPr(prhs[0]);
    vertOffset = mxGetPr(prhs[1]);
    pos = mxGetPr(prhs[2]);
    quat = mxGetPr(prhs[3]);
    bID = mxGetPr(prhs[4]);
    nb = mxGetPr(prhs[5]);  // Num bodies
    nv = mxGetPr(prhs[6]);  // Num vertices
    
    // Copy vertices 
    btScalar VERTS[ (int)nv[0] ];
    for (int v=0; v<(int)nv[0]; v++) {
        VERTS[v] = verts[v]; 
    }
    
    
    /// Collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    
    /////////////// IMPORTANT! //////////////////////////////////////
    collisionConfiguration->setConvexConvexMultipointIterations(); // Enables multiple contacts to be returned   
    /////////////////////////////////////////////////////////////////

    /// Use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
    btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

    /// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    // Keep track of the shapes, we release memory at exit.
    // Make sure to re-use collision shapes among rigid bodies whenever possible!
    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    // Create collisionWorld
    btCollisionWorld *collisionWorld = new btCollisionWorld(dispatcher, overlappingPairCache,collisionConfiguration);

    // Add all bodies to collisionWorld
    for (int b=0; b<(int)nb[0]; b++) {
        
        // Create shape
        btConvexHullShape *shape = new btConvexHullShape; 
        if (b==0)
        {
            int numVerts = (int)vertOffset[0];
            for (int v=0; v<numVerts; v++) {
                shape->addPoint( btVector3( VERTS[3*v+0], VERTS[3*v+1], VERTS[3*v+2] ) );
            }
        } else {
            int offset = (int)vertOffset[b-1];  
            int numVerts = (int)vertOffset[b] - offset;
            for (int v=offset; v<offset+numVerts; v++) {
                shape->addPoint( btVector3( VERTS[3*v+0], VERTS[3*v+1], VERTS[3*v+2] ) );
            }
        }
        
        // Add this shape to the collision shapes
        collisionShapes.push_back(shape);
        
        // Create collision object
        btCollisionObject *collisionObj = new btCollisionObject();
        collisionObj->setCollisionShape(shape); 

        // Set transformations
        btTransform objTransform;
        objTransform.setOrigin( btVector3( pos[3*b+0],pos[3*b+1],pos[3*b+2] ) );
        objTransform.setRotation(btQuaternion(quat[4*b+0], quat[4*b+1], quat[4*b+2], quat[4*b+3] ));
        collisionObj->setWorldTransform( objTransform );
        collisionObj->setCompanionId( bID[b] );  //printf("Adding body %d\n",(int)bID[b]);
        
        btVector3 P = objTransform.getOrigin();
        btQuaternion Q = objTransform.getRotation(); 
        //printf("Pos: %f, %f, %f \t Quat: %f, %f, %f, %f\n",P.x(),P.y(),P.z(),Q.x(),Q.y(),Q.z(),Q.w());
        //printf("Qagain: %f, %f, %f, %f\n",quat[4*b+0], quat[4*b+1], quat[4*b+2], quat[4*b+3]);

        // Add collision object to collisionWorld
        collisionWorld->addCollisionObject(collisionObj);
    }
    
    // Detect collision!!!
    collisionWorld->performDiscreteCollisionDetection(); 
    
    // Count total collisions
    int numManifolds = collisionWorld->getDispatcher()->getNumManifolds(); 
    if (numManifolds == 0) {
        
        ///-----cleanup_start-----

        //delete collision shapes
        for (int j=0;j<collisionShapes.size();j++)
        {
            btCollisionShape* shape = collisionShapes[j];
            collisionShapes[j] = 0;
            delete shape;
        }

        delete collisionWorld; 

        //delete broadphase
        delete overlappingPairCache;

        //delete dispatcher
        delete dispatcher;

        delete collisionConfiguration;

        //next line is optional: it will be cleared by the destructor when the array goes out of scope
        collisionShapes.clear();

        ///-----cleanup_end-----   
        
        plhs[0] = mxCreateDoubleMatrix(0,1, mxREAL);
        //double *C;
        //C = mxGetPr(plhs[0]);
        return; 
    }
    int numC = 0;
    for (int m=0; m<numManifolds; m++) {
        btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(m);
        numC += contactManifold->getNumContacts();
    }
    //printf("%d total contacts detectiond\n",numC);
    plhs[0] = mxCreateDoubleMatrix(10*numC,1, mxREAL);
    double *C;
    C = mxGetPr(plhs[0]);
    // Populate C with contact info
    int c = 0;  
    // The contact array has the following format of 10 scalars:
    // Ci = [ Cid, B1id, B2id, nx, ny, nz, px, py, pz, psi ]
    //      where n is the normal vector from B2, and p is the point of contact on B2
    for (int m=0; m<numManifolds; m++) {
        btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(m);
        int Mcontacts = contactManifold->getNumContacts();
        for (int i=0; i<Mcontacts; i++) {   // Here, we store a contact
            btManifoldPoint& pt = contactManifold->getContactPoint(i);
            btVector3 normal = pt.m_normalWorldOnB;
            btVector3 Bpt = pt.getPositionWorldOnB(); 
            
            C[10*c+0] = c+1;    // Contact ID  TODO remove this
            C[10*c+1] = contactManifold->getBody0()->getCompanionId();  // Body 1 ID
            C[10*c+2] = contactManifold->getBody1()->getCompanionId();  // Body 2 ID
            C[10*c+3] = normal.x();
            C[10*c+4] = normal.y();
            C[10*c+5] = normal.z();
            C[10*c+6] = Bpt.x();
            C[10*c+7] = Bpt.y();
            C[10*c+8] = Bpt.z();
            C[10*c+9] = pt.getDistance(); 
            c++; 
        }
    } 

    // Print out contacts
//     if (numManifolds == 0)
//         printf("No contacts found!\n");
//     else {
//         printf("NumManifolds: %d\n", numManifolds);
// 
//         for (int i=0;i<numManifolds;i++)
//         {
//             btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
//             printf("  MANIFOLD: %d\n", i);
//             int numContacts = contactManifold->getNumContacts();
//             printf("   Bodies: %d, %d\n", contactManifold->getBody0()->getCompanionId(), contactManifold->getBody1()->getCompanionId());
//             for (int j=0;j<numContacts;j++)
//             {
//                 printf("    CONTACT: %d, (%d, %d)\n", j, contactManifold->getBody0()->getCompanionId(), contactManifold->getBody1()->getCompanionId());
//                 btManifoldPoint& pt = contactManifold->getContactPoint(j);
//                 btVector3 ptA = pt.getPositionWorldOnA();
//                 btVector3 ptB = pt.getPositionWorldOnB();
//                 printf("      pA = (%f,%f,%f)\n",ptA.x(),ptA.y(),ptA.z());
//                 printf("      pB = (%f,%f,%f)\n",ptB.x(),ptB.y(),ptB.z());
//                 btVector3 n = pt.m_normalWorldOnB; 
//                 printf("      psi_n = %f,  Bnorm = (%f,%f,%f)\n",pt.getDistance(), n.x(), n.y(), n.z());
// 
//             }
//         }
//     }


    //cleanup in the reverse order of creation/initialization

    ///-----cleanup_start-----

    //delete collision shapes
    for (int j=0;j<collisionShapes.size();j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }

    delete collisionWorld; 

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;

    //next line is optional: it will be cleared by the destructor when the array goes out of scope
    collisionShapes.clear();

    ///-----cleanup_end-----   
    
    
}



















