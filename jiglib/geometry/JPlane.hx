package jiglib.geometry;

import jiglib.geometry.JSegment;

import jiglib.data.CollOutData;
import jiglib.math.*;
import jiglib.physics.PhysicsState;
import jiglib.physics.RigidBody;
import jiglib.plugin.ISkin3D;

class JPlane extends RigidBody
{
    public var normal(get, never) : Vector3D;
    public var distance(get, never) : Float;

    private var _initNormal : Vector3D;
    private var _normal : Vector3D;
    private var _distance : Float;
    
    public function new(skin : ISkin3D, initNormal : Vector3D = null)
    {
        super(skin);
        
        _initNormal = (initNormal != null) ? initNormal.clone() : new Vector3D(0, 0, -1);
        _normal = _initNormal.clone();
        
        _distance = 0;
        this.movable = false;
        
        var huge : Float = JMath3D.NUM_HUGE;
        _boundingBox.minPos = new Vector3D(-huge, -huge, -huge);
        _boundingBox.maxPos = new Vector3D(huge, huge, huge);
        
        _type = "PLANE";
    }
    
    private function get_normal() : Vector3D
    {
        return _normal;
    }
    
    private function get_distance() : Float
    {
        return _distance;
    }
    
    public function pointPlaneDistance(pt : Vector3D) : Float
    {
        return _normal.dotProduct(pt) - _distance;
    }
    
    override public function segmentIntersect(out : CollOutData, seg : JSegment, state : PhysicsState) : Bool
    {
        out.frac = 0;
        out.position = new Vector3D();
        out.normal = new Vector3D();
        
        var frac : Float = 0;
        var t : Float;
        var denom : Float;
        
        denom = _normal.dotProduct(seg.delta);
        if (Math.abs(denom) > JMath3D.NUM_TINY) 
        {
            t = -1 * (_normal.dotProduct(seg.origin) - _distance) / denom;
            
            if (t < 0 || t > 1) 
            {
                return false;
            }
            else 
            {
                frac = t;
                out.frac = frac;
                out.position = seg.getPoint(frac);
                out.normal = _normal.clone();
                out.normal.normalize();
                return true;
            }
        }
        else 
        {
            return false;
        }
    }
    
    override private function updateState() : Void
    {
        super.updateState();
        
        _normal = _currState.orientation.transformVector(_initNormal);
        _distance = _currState.position.dotProduct(_normal);
    }
}
