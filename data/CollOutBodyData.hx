package jiglib.data;

import jiglib.data.CollOutData;

import jiglib.math.Vector3D;
import jiglib.physics.RigidBody;

class CollOutBodyData extends CollOutData
{
    public var rigidBody : RigidBody;
    
    public function new(frac : Float = 0, position : Vector3D = null, normal : Vector3D = null, rigidBody : RigidBody = null)
    {
        super(frac, position, normal);
        this.rigidBody = rigidBody;
    }
}
