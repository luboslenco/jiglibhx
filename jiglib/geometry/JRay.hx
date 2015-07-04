package jiglib.geometry;

import jiglib.math.*;

class JRay
{
    public var origin : Vector3D;
    public var dir : Vector3D;
    
    public function new(_origin : Vector3D, _dir : Vector3D)
    {
        this.origin = _origin;
        this.dir = _dir;
    }
    
    public function getOrigin(t : Float) : Vector3D
    {
        return origin.add(JNumber3D.getScaleVector(dir, t));
    }
}
