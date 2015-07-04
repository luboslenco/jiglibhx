package jiglib.geometry;

import jiglib.geometry.JSegment;

import jiglib.data.CollOutData;
import jiglib.math.*;
import jiglib.physics.PhysicsState;
import jiglib.physics.RigidBody;
import jiglib.plugin.ISkin3D;

class JCapsule extends RigidBody
{
    public var radius(get, set) : Float;
    public var length(get, set) : Float;

    
    private var _length : Float;  //cylinder height  
    private var _radius : Float;  //radius of sphere  
    
    public function new(skin : ISkin3D, r : Float, l : Float)
    {
        super(skin);
        _type = "CAPSULE";
        _radius = r;
        _length = l;
        _boundingSphere = getBoundingSphere(r, l);
        mass = 1;
        updateBoundingBox();
    }
    
    private function set_radius(r : Float) : Float{
        _radius = r;
        _boundingSphere = getBoundingSphere(_radius, _length);
        setInertia(getInertiaProperties(mass));
        updateBoundingBox();
        setActive();
        return r;
    }
    private function get_radius() : Float{
        return _radius;
    }
    
    private function set_length(l : Float) : Float{
        _length = l;
        _boundingSphere = getBoundingSphere(_radius, _length);
        setInertia(getInertiaProperties(mass));
        updateBoundingBox();
        setActive();
        return l;
    }
    private function get_length() : Float{
        return _length;
    }
    
    public function getBottomPos(state : PhysicsState) : Vector3D{
        return state.position.add(JNumber3D.getScaleVector(state.getOrientationCols()[1], -_length / 2));
    }
    
    public function getEndPos(state : PhysicsState) : Vector3D{
        return state.position.add(JNumber3D.getScaleVector(state.getOrientationCols()[1], _length / 2));
    }
    
    override public function segmentIntersect(out : CollOutData, seg : JSegment, state : PhysicsState) : Bool
    {
        out.frac = 0;
        out.position = new Vector3D();
        out.normal = new Vector3D();
        
        var kss : Float;
        var radiusSq : Float;
        var kee : Float;
        var kes : Float;
        var kgs : Float;
        var keg : Float;
        var kgg : Float;
        var distSq : Float;
        var a : Float;
        var b : Float;
        var c : Float;
        var blah : Float;
        var t : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        var Ks : Vector3D;
        var Ke : Vector3D;
        var Kg : Vector3D;
        Ks = seg.delta;
        kss = Ks.dotProduct(Ks);
        radiusSq = _radius * _radius;
        
        var cols : Array<Vector3D> = state.getOrientationCols();
        var cylinderAxis : JSegment = new JSegment(getBottomPos(state), cols[1]);
        Ke = cylinderAxis.delta;
        Kg = cylinderAxis.origin.subtract(seg.origin);
        kee = Ke.dotProduct(Ke);
        if (Math.abs(kee) < tiny) 
        {
            return false;
        }
        
        kes = Ke.dotProduct(Ks);
        kgs = Kg.dotProduct(Ks);
        keg = Ke.dotProduct(Kg);
        kgg = Kg.dotProduct(Kg);
        
        distSq = Kg.subtract(JNumber3D.getDivideVector(JNumber3D.getScaleVector(Ke, keg), kee)).lengthSquared;
        if (distSq < radiusSq) 
        {
            out.frac = 0;
            out.position = seg.origin.clone();
            out.normal = out.position.subtract(getBottomPos(state));
            out.normal = out.normal.subtract(JNumber3D.getScaleVector(cols[1], out.normal.dotProduct(cols[1])));
            out.normal.normalize();
            return true;
        }
        
        a = kee * kss - (kes * kes);
        if (Math.abs(a) < tiny) 
        {
            return false;
        }
        b = 2 * (keg * kes - kee * kgs);
        c = kee * (kgg - radiusSq) - (keg * keg);
        blah = (b * b) - 4 * a * c;
        if (blah < 0) 
        {
            return false;
        }
        t = (-b - Math.sqrt(blah)) / (2 * a);
        if (t < 0 || t > 1) 
        {
            return false;
        }
        out.frac = t;
        out.position = seg.getPoint(t);
        out.normal = out.position.subtract(getBottomPos(state));
        out.normal = out.normal.subtract(JNumber3D.getScaleVector(cols[1], out.normal.dotProduct(cols[1])));
        out.normal.normalize();
        return true;
    }
    
    override public function getInertiaProperties(m : Float) : Matrix3D{
        var cylinderMass : Float;
        var Ixx : Float;
        var Iyy : Float;
        var Izz : Float;
        var endMass : Float;
        cylinderMass = m * Math.PI * _radius * _radius * _length / getVolume();
        Ixx = 0.25 * cylinderMass * _radius * _radius + (1 / 12) * cylinderMass * _length * _length;
        Iyy = 0.5 * cylinderMass * _radius * _radius;
        Izz = Ixx;
        
        endMass = m - cylinderMass;
        Ixx += (0.4 * endMass * _radius * _radius + endMass * Math.pow(0.5 * _length, 2));
        Iyy += (0.2 * endMass * _radius * _radius);
        Izz += (0.4 * endMass * _radius * _radius + endMass * Math.pow(0.5 * _length, 2));
        
        return JMatrix3D.getScaleMatrix(Ixx, Iyy, Izz);
    }
    
    override private function updateBoundingBox() : Void{
        _boundingBox.clear();
        _boundingBox.addCapsule(this);
    }
    
    private function getBoundingSphere(r : Float, l : Float) : Float{
        return Math.sqrt(Math.pow(l / 2, 2) + r * r) + r;
    }
    
    private function getVolume() : Float{
        return (4 / 3) * Math.PI * _radius * _radius * _radius + _length * Math.PI * _radius * _radius;
    }
}


