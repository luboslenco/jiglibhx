package jiglib.geometry;

import jiglib.data.CollOutData;
import jiglib.math.*;
import jiglib.physics.PhysicsState;
import jiglib.physics.RigidBody;
import jiglib.plugin.ISkin3D;

class JSphere extends RigidBody
{
    public var radius(get, set) : Float;

    
    public var name : String;
    private var _radius : Float;
    
    public function new(skin : ISkin3D, r : Float)
    {
        super(skin);
        _type = "SPHERE";
        _radius = r;
        _boundingSphere = _radius;
        mass = 1;
        updateBoundingBox();
    }
    
    private function set_radius(r : Float) : Float
    {
        _radius = r;
        _boundingSphere = _radius;
        setInertia(getInertiaProperties(mass));
        setActive();
        updateBoundingBox();
        return r;
    }
    
    private function get_radius() : Float
    {
        return _radius;
    }
    
    override public function segmentIntersect(out : CollOutData, seg : JSegment, state : PhysicsState) : Bool
    {
        out.frac = 0;
        out.position = new Vector3D();
        out.normal = new Vector3D();
        
        var frac : Float = 0;
        var radiusSq : Float;
        var rSq : Float;
        var sDotr : Float;
        var sSq : Float;
        var sigma : Float;
        var sigmaSqrt : Float;
        var lambda1 : Float;
        var lambda2 : Float;
        var r : Vector3D;
        var s : Vector3D;
        r = seg.delta;
        s = seg.origin.subtract(state.position);
        
        radiusSq = _radius * _radius;
        rSq = r.lengthSquared;
        if (rSq < radiusSq) 
        {
            out.frac = 0;
            out.position = seg.origin.clone();
            out.normal = out.position.subtract(state.position);
            out.normal.normalize();
            return true;
        }
        
        sDotr = s.dotProduct(r);
        sSq = s.lengthSquared;
        sigma = sDotr * sDotr - rSq * (sSq - radiusSq);
        if (sigma < 0) 
        {
            return false;
        }
        sigmaSqrt = Math.sqrt(sigma);
        lambda1 = (-sDotr - sigmaSqrt) / rSq;
        lambda2 = (-sDotr + sigmaSqrt) / rSq;
        if (lambda1 > 1 || lambda2 < 0) 
        {
            return false;
        }
        frac = Math.max(lambda1, 0);
        out.frac = frac;
        out.position = seg.getPoint(frac);
        out.normal = out.position.subtract(state.position);
        out.normal.normalize();
        return true;
    }
    
    override public function getInertiaProperties(m : Float) : Matrix3D
    {
        var Ixx : Float = 0.4 * m * _radius * _radius;
        return JMatrix3D.getScaleMatrix(Ixx, Ixx, Ixx);
    }
    
    override private function updateBoundingBox() : Void{
        _boundingBox.clear();
        _boundingBox.addSphere(this);
    }
}

