package jiglib.collision;

import jiglib.math.*;

class CollPointInfo
{
    public var initialPenetration : Float;
    public var r0 : Vector3D;
    public var r1 : Vector3D;
    public var position : Vector3D;
    
    public var minSeparationVel : Float = 0;
    public var denominator : Float = 0;
    
    public var accumulatedNormalImpulse : Float = 0;
    public var accumulatedNormalImpulseAux : Float = 0;
    public var accumulatedFrictionImpulse : Vector3D = new Vector3D();

    public function new()
    {
    }
}
