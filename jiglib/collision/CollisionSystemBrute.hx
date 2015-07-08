package jiglib.collision;


import jiglib.physics.RigidBody;

/**
	 * <code>CollisionSystemBrute</code> checks every rigidbody against each other. 
	 * For small scenes this is	as fast or even faster as CollisionSystemGrid.
	 * 
	 */
class CollisionSystemBrute extends CollisionSystemAbstract
{
    
    public function new()
    {
        super();
    }
    
    // Detects collisions between the all bodies
    override public function detectAllCollisions(bodies : Array<RigidBody>, collArr : Array<CollisionInfo>) : Void
    {
        var info : CollDetectInfo;
        var fu : CollDetectFunctor;
        var bodyID : Int;
        var bodyType : String;
        _numCollisionsChecks = 0;
        for (_body in bodies)
        {
            if (!_body.isActive) continue;
            
            bodyID = _body.id;
            bodyType = _body.type;
            for (_collBody in collBody)
            {
                if (_body == _collBody) 
                {
                    continue;
                }
                
                if (_collBody.isActive && bodyID > _collBody.id) 
                {
                    continue;
                }
                
                if (checkCollidables(_body, _collBody) && detectionFunctors[bodyType + "_" + _collBody.type] != null) 
                {
                    info = new CollDetectInfo();
                    info.body0 = _body;
                    info.body1 = _collBody;
                    fu = detectionFunctors[info.body0.type + "_" + info.body1.type];
                    fu.collDetect(info, collArr);
                    _numCollisionsChecks += 1;
                }
            }
        }
    }
}
