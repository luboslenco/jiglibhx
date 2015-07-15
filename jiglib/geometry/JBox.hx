package jiglib.geometry;

import jiglib.geometry.JSegment;

import jiglib.data.CollOutData;
import jiglib.data.EdgeData;
import jiglib.data.SpanData;
import jiglib.math.*;
import jiglib.physics.PhysicsState;
import jiglib.physics.RigidBody;
import jiglib.plugin.ISkin3D;

class JBox extends RigidBody
{
    public var sideLengths(get, set) : Vector3D;
    public var edges(get, never) : Array<EdgeData>;

    private var _sideLengths : Vector3D;
    private var _points : Array<Vector3D>;
    private var _edges : Array<EdgeData> = [
                new EdgeData(0, 1), new EdgeData(0, 2), new EdgeData(0, 6), 
                new EdgeData(2, 3), new EdgeData(2, 4), new EdgeData(6, 7), 
                new EdgeData(6, 4), new EdgeData(1, 3), new EdgeData(1, 7), 
                new EdgeData(3, 5), new EdgeData(7, 5), new EdgeData(4, 5)];
    
    private var _face : Array<Array<Float>> = [
                [6, 7, 1, 0], [5, 4, 2, 3], 
                [3, 1, 7, 5], [4, 6, 0, 2], 
                [1, 3, 2, 0], [7, 6, 4, 5]];
    
    public function new(skin:ISkin3D, width : Float, depth : Float, height : Float)
    {
        super(skin);
        _type = "BOX";
        
        _sideLengths = new Vector3D(width, height, depth);
        _boundingSphere = 0.5 * _sideLengths.length;
        initPoint();
        mass = 1;
        updateBoundingBox();
    }
    
    private function initPoint() : Void
    {
        var halfSide : Vector3D = getHalfSideLengths();
        _points = new Array<Vector3D>();
        _points.push( new Vector3D(halfSide.x, -halfSide.y, halfSide.z) );
        _points.push( new Vector3D(halfSide.x, halfSide.y, halfSide.z) );
        _points.push( new Vector3D(-halfSide.x, -halfSide.y, halfSide.z) );
        _points.push( new Vector3D(-halfSide.x, halfSide.y, halfSide.z) );
        _points.push( new Vector3D(-halfSide.x, -halfSide.y, -halfSide.z) );
        _points.push( new Vector3D(-halfSide.x, halfSide.y, -halfSide.z) );
        _points.push( new Vector3D(halfSide.x, -halfSide.y, -halfSide.z) );
        _points.push( new Vector3D(halfSide.x, halfSide.y, -halfSide.z) );
    }
    
    private function set_sideLengths(size : Vector3D) : Vector3D
    {
        _sideLengths = size.clone();
        _boundingSphere = 0.5 * _sideLengths.length;
        initPoint();
        setInertia(getInertiaProperties(mass));
        setActive();
        updateBoundingBox();
        return size;
    }
    
    //Returns the full side lengths
    private function get_sideLengths() : Vector3D
    {
        return _sideLengths;
    }
    
    private function get_edges() : Array<EdgeData>
    {
        return _edges;
    }
    
    public function getVolume() : Float
    {
        return (_sideLengths.x * _sideLengths.y * _sideLengths.z);
    }
    
    public function getSurfaceArea() : Float
    {
        return 2 * (_sideLengths.x * _sideLengths.y + _sideLengths.x * _sideLengths.z + _sideLengths.y * _sideLengths.z);
    }
    
    // Returns the half-side lengths
    public function getHalfSideLengths() : Vector3D
    {
        return JNumber3D.getScaleVector(_sideLengths, 0.5);
    }
    
    // Gets the minimum and maximum extents of the box along the axis, relative to the centre of the box.
    public function getSpan(axis : Vector3D) : SpanData
    {
        var s : Float;
        var u : Float;
        var d : Float;
        var r : Float;
        var p : Float;
        var cols : Array<Vector3D> = currentState.getOrientationCols();
        var obj : SpanData = new SpanData();
        s = Math.abs(axis.dotProduct(cols[0])) * (0.5 * _sideLengths.x);
        u = Math.abs(axis.dotProduct(cols[1])) * (0.5 * _sideLengths.y);
        d = Math.abs(axis.dotProduct(cols[2])) * (0.5 * _sideLengths.z);
        r = s + u + d;
        p = currentState.position.dotProduct(axis);
        obj.min = p - r;
        obj.max = p + r;
        
        return obj;
    }
    
    // Gets the corner points in world space
    public function getCornerPoints(state : PhysicsState) : Array<Vector3D>
    {
        var arr : Array<Vector3D> = new Array<Vector3D>();
        
        var transform : Matrix3D = JMatrix3D.getTranslationMatrix(state.position.x, state.position.y, state.position.z);
        transform = JMatrix3D.getAppendMatrix3D(state.orientation, transform);
        
        for (_point in _points){
            arr.push( transform.transformVector(_point) );
        }
        
        return arr;
    }
    
    // Gets the corner points in another box space
    public function getCornerPointsInBoxSpace(thisState : PhysicsState, boxState : PhysicsState) : Array<Vector3D>{
        
        var max : Matrix3D;
        var orient : Matrix3D;
        var transform : Matrix3D;
        
        max = JMatrix3D.getTransposeMatrix(boxState.orientation);
        var pos : Vector3D = thisState.position.subtract(boxState.position);
        pos = max.transformVector(pos);
        
        orient = JMatrix3D.getAppendMatrix3D(thisState.orientation, max);
        
        var arr : Array<Vector3D> = new Array<Vector3D>();
        
        transform = JMatrix3D.getTranslationMatrix(pos.x, pos.y, pos.z);
        transform = JMatrix3D.getAppendMatrix3D(orient, transform);
        
        for (_point in _points)
            arr.push( transform.transformVector(_point) );
        
        return arr;
    }
    
    public function getSqDistanceToPoint(state : PhysicsState, closestBoxPoint : Array<Vector3D>, point : Vector3D) : Float
    {
        var _closestBoxPoint : Vector3D;
        var halfSideLengths : Vector3D;
        var delta : Float = 0;
        var sqDistance : Float = 0;
        
        _closestBoxPoint = point.subtract(state.position);
        _closestBoxPoint = JMatrix3D.getTransposeMatrix(state.orientation).transformVector(_closestBoxPoint);
        
        halfSideLengths = getHalfSideLengths();
        
        if (_closestBoxPoint.x < -halfSideLengths.x) 
        {
            delta = _closestBoxPoint.x + halfSideLengths.x;
            sqDistance += (delta * delta);
            _closestBoxPoint.x = -halfSideLengths.x;
        }
        else if (_closestBoxPoint.x > halfSideLengths.x) 
        {
            delta = _closestBoxPoint.x - halfSideLengths.x;
            sqDistance += (delta * delta);
            _closestBoxPoint.x = halfSideLengths.x;
        }
        
        if (_closestBoxPoint.y < -halfSideLengths.y) 
        {
            delta = _closestBoxPoint.y + halfSideLengths.y;
            sqDistance += (delta * delta);
            _closestBoxPoint.y = -halfSideLengths.y;
        }
        else if (_closestBoxPoint.y > halfSideLengths.y) 
        {
            delta = _closestBoxPoint.y - halfSideLengths.y;
            sqDistance += (delta * delta);
            _closestBoxPoint.y = halfSideLengths.y;
        }
        
        if (_closestBoxPoint.z < -halfSideLengths.z) 
        {
            delta = _closestBoxPoint.z + halfSideLengths.z;
            sqDistance += (delta * delta);
            _closestBoxPoint.z = -halfSideLengths.z;
        }
        else if (_closestBoxPoint.z > halfSideLengths.z) 
        {
            delta = (_closestBoxPoint.z - halfSideLengths.z);
            sqDistance += (delta * delta);
            _closestBoxPoint.z = halfSideLengths.z;
        }
        _closestBoxPoint = state.orientation.transformVector(_closestBoxPoint);
        closestBoxPoint[0] = state.position.add(_closestBoxPoint);
        return sqDistance;
    }
    
    // Returns the distance from the point to the box, (-ve if the
    // point is inside the box), and optionally the closest point on the box.
    public function getDistanceToPoint(state : PhysicsState, closestBoxPoint : Array<Vector3D>, point : Vector3D) : Float
    {
        return Math.sqrt(getSqDistanceToPoint(state, closestBoxPoint, point));
    }
    
    public function pointIntersect(pos : Vector3D) : Bool
    {
        var p : Vector3D;
        var h : Vector3D;
        var dirVec : Vector3D;
        
        p = pos.subtract(currentState.position);
        h = JNumber3D.getScaleVector(_sideLengths, 0.5);
        
        var cols : Array<Vector3D> = currentState.getOrientationCols();
        for (dir in 0...3){
            dirVec = cols[dir].clone();
            dirVec.normalize();
            if (Math.abs(dirVec.dotProduct(p)) > JNumber3D.toArray(h)[dir] + JMath3D.NUM_TINY) 
            {
                return false;
            }
        }
        return true;
    }
    
    override public function segmentIntersect(out : CollOutData, seg : JSegment, state : PhysicsState) : Bool
    {
        out.frac = 0;
        out.position = new Vector3D();
        out.normal = new Vector3D();
        
        var tiny : Float = JMath3D.NUM_TINY;
        var huge : Float = JMath3D.NUM_HUGE;
        var frac : Float;
        var min : Float;
        var max : Float;
        var dirMin : Float = 0;
        var dirMax : Float = 0;
        var dir : Float = 0;
        var e : Float;
        var f : Float;
        var t : Float;
        var t1 : Float;
        var t2 : Float;
        var directionVectorNumber : Float;
        var p : Vector3D;
        var h : Vector3D;
        
        frac = huge;
        min = -huge;
        max = huge;
        p = state.position.subtract(seg.origin);
        h = JNumber3D.getScaleVector(_sideLengths, 0.5);
        
        var orientationCol : Array<Vector3D> = state.getOrientationCols();
        var directionVectorArray : Array<Float> = JNumber3D.toArray(h);
        for (dir in 0...3){
            directionVectorNumber = directionVectorArray[dir];
            e = orientationCol[dir].dotProduct(p);
            f = orientationCol[dir].dotProduct(seg.delta);
            if (Math.abs(f) > tiny) 
            {
                t1 = (e + directionVectorNumber) / f;
                t2 = (e - directionVectorNumber) / f;
                if (t1 > t2) 
                {
                    t = t1;
                    t1 = t2;
                    t2 = t;
                }
                if (t1 > min) 
                {
                    min = t1;
                    dirMin = dir;
                }
                if (t2 < max) 
                {
                    max = t2;
                    dirMax = dir;
                }
                if (min > max) 
                    return false;
                if (max < 0) 
                    return false;
            }
            else if (-e - directionVectorNumber > 0 || -e + directionVectorNumber < 0) 
            {
                return false;
            }
        }
        
        if (min > 0) 
        {
            dir = dirMin;
            frac = min;
        }
        else 
        {
            dir = dirMax;
            frac = max;
        }
        if (frac < 0) 
            frac = 0;
        /*if (frac > 1)
            frac = 1;*/
        if (frac > 1 - tiny) 
        {
            return false;
        }
        out.frac = frac;
        out.position = seg.getPoint(frac);
        if (orientationCol[Std.int(dir)].dotProduct(seg.delta) < 0) 
        {
            out.normal = JNumber3D.getScaleVector(orientationCol[Std.int(dir)], -1);
        }
        else 
        {
            out.normal = orientationCol[Std.int(dir)];
        }
        return true;
    }
    
    override public function getInertiaProperties(m : Float) : Matrix3D
    {
        return JMatrix3D.getScaleMatrix(
                (m / 12) * (_sideLengths.y * _sideLengths.y + _sideLengths.z * _sideLengths.z),
                (m / 12) * (_sideLengths.x * _sideLengths.x + _sideLengths.z * _sideLengths.z),
                (m / 12) * (_sideLengths.x * _sideLengths.x + _sideLengths.y * _sideLengths.y));
    }
    
    override private function updateBoundingBox() : Void{
        _boundingBox.clear();
        _boundingBox.addBox(this);
    }
}
