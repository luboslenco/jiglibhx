package jiglib.collision;

import jiglib.collision.CollPointInfo;

import jiglib.physics.MaterialProperties;
import jiglib.math.Vector3D;

class CollisionInfo
{
    public var mat : MaterialProperties = new MaterialProperties();
    public var objInfo : CollDetectInfo;
    public var dirToBody : Vector3D;
    public var pointInfo : Array<CollPointInfo>;
    public var satisfied : Bool;

    public function new()
    {
    }
}
