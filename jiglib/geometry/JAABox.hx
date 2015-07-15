package jiglib.geometry;

import jiglib.geometry.JBox;
import jiglib.geometry.JCapsule;
import jiglib.geometry.JSegment;
import jiglib.geometry.JSphere;

import jiglib.data.EdgeData;
import jiglib.math.JNumber3D;
import jiglib.math.JMath3D;
import jiglib.math.Vector3D;

// An axis-aligned box
class JAABox
{
    public var sideLengths(get, never) : Vector3D;
    public var centrePos(get, never) : Vector3D;
    public var edges(get, never) : Array<EdgeData>;

    
    public var minPos : Vector3D;
    public var maxPos : Vector3D;
    
    public function new()
    {
        clear();
    }
    
    private function get_sideLengths() : Vector3D{
        var pos : Vector3D = maxPos.clone();
        pos = pos.subtract(minPos);
        return pos;
    }
    
    private function get_centrePos() : Vector3D{
        var pos : Vector3D = minPos.clone();
        return JNumber3D.getScaleVector(pos.add(maxPos), 0.5);
    }
    
    public function getAllPoints() : Array<Vector3D>{
        var center : Vector3D;
        var halfSide : Vector3D;
        var points : Array<Vector3D>;
        center = this.centrePos;
        halfSide = JNumber3D.getScaleVector(this.sideLengths, 0.5);
        points = new Array<Vector3D>();
        points.push( center.add(new Vector3D(halfSide.x, -halfSide.y, halfSide.z)) );
        points.push( center.add(new Vector3D(halfSide.x, halfSide.y, halfSide.z)) );
        points.push( center.add(new Vector3D(-halfSide.x, -halfSide.y, halfSide.z)) );
        points.push( center.add(new Vector3D(-halfSide.x, halfSide.y, halfSide.z)) );
        points.push( center.add(new Vector3D(-halfSide.x, -halfSide.y, -halfSide.z)) );
        points.push( center.add(new Vector3D(-halfSide.x, halfSide.y, -halfSide.z)) );
        points.push( center.add(new Vector3D(halfSide.x, -halfSide.y, -halfSide.z)) );
        points.push( center.add(new Vector3D(halfSide.x, halfSide.y, -halfSide.z)) );
        
        return points;
    }
    
    private function get_edges() : Array<EdgeData>{
        return [
                new EdgeData(0, 1), new EdgeData(0, 2), new EdgeData(0, 6), 
                new EdgeData(2, 3), new EdgeData(2, 4), new EdgeData(6, 7), 
                new EdgeData(6, 4), new EdgeData(1, 3), new EdgeData(1, 7), 
                new EdgeData(3, 5), new EdgeData(7, 5), new EdgeData(4, 5)];
    }
    
    public function getRadiusAboutCentre() : Float{
        return 0.5 * (maxPos.subtract(minPos).length);
    }
    
    public function move(delta : Vector3D) : Void{
        minPos.add(delta);
        maxPos.add(delta);
    }
    
    public function clear() : Void{
        var huge : Float = JMath3D.NUM_HUGE;
        minPos = new Vector3D(huge, huge, huge);
        maxPos = new Vector3D(-huge, -huge, -huge);
    }
    
    public function clone() : JAABox{
        var aabb : JAABox = new JAABox();
        aabb.minPos = this.minPos.clone();
        aabb.maxPos = this.maxPos.clone();
        return aabb;
    }
    
    
    
    public function addPoint(pos : Vector3D) : Void{
        var tiny : Float = JMath3D.NUM_TINY;
        if (pos.x < minPos.x)             minPos.x = pos.x - tiny;
        if (pos.x > maxPos.x)             maxPos.x = pos.x + tiny;
        if (pos.y < minPos.y)             minPos.y = pos.y - tiny;
        if (pos.y > maxPos.y)             maxPos.y = pos.y + tiny;
        if (pos.z < minPos.z)             minPos.z = pos.z - tiny;
        if (pos.z > maxPos.z)             maxPos.z = pos.z + tiny;
    }
    
    public function addBox(box : JBox) : Void{
        var pts : Array<Vector3D> = box.getCornerPoints(box.currentState);
        addPoint(pts[0]);
        addPoint(pts[1]);
        addPoint(pts[2]);
        addPoint(pts[3]);
        addPoint(pts[4]);
        addPoint(pts[5]);
        addPoint(pts[6]);
        addPoint(pts[7]);
    }
    
    
    // todo: the extra if doesn't make sense and bugs the size, also shouldn't this called once and if scaled?
    public function addSphere(sphere : JSphere) : Void{
        //if (sphere.currentState.position.x - sphere.radius < _minPos.x) {
        minPos.x = (sphere.currentState.position.x - sphere.radius) - 1;
        //}
        //if (sphere.currentState.position.x + sphere.radius > _maxPos.x) {
        maxPos.x = (sphere.currentState.position.x + sphere.radius) + 1;
        //}
        
        //if (sphere.currentState.position.y - sphere.radius < _minPos.y) {
        minPos.y = (sphere.currentState.position.y - sphere.radius) - 1;
        //}
        //if (sphere.currentState.position.y + sphere.radius > _maxPos.y) {
        maxPos.y = (sphere.currentState.position.y + sphere.radius) + 1;
        //}
        
        //if (sphere.currentState.position.z - sphere.radius < _minPos.z) {
        minPos.z = (sphere.currentState.position.z - sphere.radius) - 1;
        //}
        //if (sphere.currentState.position.z + sphere.radius > _maxPos.z) {
        maxPos.z = (sphere.currentState.position.z + sphere.radius) + 1;
    }
    
    public function addCapsule(capsule : JCapsule) : Void{
        var pos : Vector3D = capsule.getBottomPos(capsule.currentState);
        if (pos.x - capsule.radius < minPos.x) {
            minPos.x = (pos.x - capsule.radius) - 1;
        }
        if (pos.x + capsule.radius > maxPos.x) {
            maxPos.x = (pos.x + capsule.radius) + 1;
        }
        
        if (pos.y - capsule.radius < minPos.y) {
            minPos.y = (pos.y - capsule.radius) - 1;
        }
        if (pos.y + capsule.radius > maxPos.y) {
            maxPos.y = (pos.y + capsule.radius) + 1;
        }
        
        if (pos.z - capsule.radius < minPos.z) {
            minPos.z = (pos.z - capsule.radius) - 1;
        }
        if (pos.z + capsule.radius > maxPos.z) {
            maxPos.z = (pos.z + capsule.radius) + 1;
        }
        
        pos = capsule.getEndPos(capsule.currentState);
        if (pos.x - capsule.radius < minPos.x) {
            minPos.x = (pos.x - capsule.radius) - 1;
        }
        if (pos.x + capsule.radius > maxPos.x) {
            maxPos.x = (pos.x + capsule.radius) + 1;
        }
        
        if (pos.y - capsule.radius < minPos.y) {
            minPos.y = (pos.y - capsule.radius) - 1;
        }
        if (pos.y + capsule.radius > maxPos.y) {
            maxPos.y = (pos.y + capsule.radius) + 1;
        }
        
        if (pos.z - capsule.radius < minPos.z) {
            minPos.z = (pos.z - capsule.radius) - 1;
        }
        if (pos.z + capsule.radius > maxPos.z) {
            maxPos.z = (pos.z + capsule.radius) + 1;
        }
    }
    
    public function addSegment(seg : JSegment) : Void{
        addPoint(seg.origin);
        addPoint(seg.getEnd());
    }
    
    public function overlapTest(box : JAABox) : Bool{
        return ((
        (minPos.z >= box.maxPos.z) ||
        (maxPos.z <= box.minPos.z) ||
        (minPos.y >= box.maxPos.y) ||
        (maxPos.y <= box.minPos.y) ||
        (minPos.x >= box.maxPos.x) ||
        (maxPos.x <= box.minPos.x))) ? false : true;
    }
    
    public function isPointInside(pos : Vector3D) : Bool{
        return ((pos.x >= minPos.x) &&
        (pos.x <= maxPos.x) &&
        (pos.y >= minPos.y) &&
        (pos.y <= maxPos.y) &&
        (pos.z >= minPos.z) &&
        (pos.z <= maxPos.z));
    }
    
    public function segmentAABoxOverlap(seg : JSegment) : Bool{
        var jDir : Int;
        var kDir : Int;
        var i : Int;
        var iFace : Int;
        var frac : Float;
        var dist0 : Float;
        var dist1 : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        
        var pt : Array<Float>;
        var minPosArr : Array<Float>;
        var maxPosArr : Array<Float>;
        var p0 : Array<Float>;
        var p1 : Array<Float>;
        var faceOffsets : Array<Float>;
        minPosArr = JNumber3D.toArray(minPos);
        maxPosArr = JNumber3D.toArray(maxPos);
        p0 = JNumber3D.toArray(seg.origin);
        p1 = JNumber3D.toArray(seg.getEnd());
        for (i in 0...3){
            jDir = (i + 1) % 3;
            kDir = (i + 2) % 3;
            faceOffsets = [minPosArr[i], maxPosArr[i]];
            
            for (iFace in 0...2){
                dist0 = p0[i] - faceOffsets[iFace];
                dist1 = p1[i] - faceOffsets[iFace];
                frac = -1;
                if (dist0 * dist1 < -tiny) 
                    frac = -dist0 / (dist1 - dist0)
                else if (Math.abs(dist0) < tiny) 
                    frac = 0
                else if (Math.abs(dist1) < tiny) 
                    frac = 1;
                
                if (frac >= 0) {
                    pt = JNumber3D.toArray(seg.getPoint(frac));
                    if ((pt[jDir] > minPosArr[jDir] - tiny) &&
                        (pt[jDir] < maxPosArr[jDir] + tiny) &&
                        (pt[kDir] > minPosArr[kDir] - tiny) &&
                        (pt[kDir] < maxPosArr[kDir] + tiny)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
}
