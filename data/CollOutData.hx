package jiglib.data;

import jiglib.math.Vector3D;
import jiglib.physics.RigidBody;

class CollOutData
{
    public var frac : Float;
    public var position : Vector3D;
    public var normal : Vector3D;
    
    public function new(frac : Float = 0, position : Vector3D = null, normal : Vector3D = null)
    {
        this.frac = (Math.isNaN(frac)) ? 0 : frac;
        this.position = (position != null) ? position : new Vector3D();
        this.normal = (normal != null) ? normal : new Vector3D();
    }
}
