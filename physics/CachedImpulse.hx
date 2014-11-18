package jiglib.physics;

import jiglib.math.Vector3D;

class CachedImpulse
{
    
    public var normalImpulse : Float;
    public var normalImpulseAux : Float;
    public var frictionImpulse : Vector3D;
    
    public function new(_normalImpulse : Float, _normalImpulseAux : Float, _frictionImpulse : Vector3D)
    {
        this.normalImpulse = _normalImpulse;
        this.normalImpulseAux = _normalImpulseAux;
        this.frictionImpulse = _frictionImpulse;
    }
}
