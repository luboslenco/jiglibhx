package jiglib.collision;

import jiglib.collision.CollDetectFunctor;
import jiglib.collision.CollDetectInfo;
import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;
//import jiglib.collision.JBox;
//import jiglib.collision.JSegment;

import jiglib.cof.JConfig;
import jiglib.data.EdgeData;
import jiglib.data.SpanData;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;
import jiglib.physics.PhysicsState;

class CollDetectBoxBox extends CollDetectFunctor
{
    private inline static var MAX_SUPPORT_VERTS : Float = 10;
    private var combinationDist : Float;
    
    public function new()
    {
        super();
        name = "BoxBox";
        type0 = "BOX";
        type1 = "BOX";
    }
    
    //Returns true if disjoint.  Returns false if intersecting
    private function disjoint(out : SpanData, axis : Vector3D, box0 : JBox, box1 : JBox) : Bool
    {
        var obj0 : SpanData = box0.getSpan(axis);
        var obj1 : SpanData = box1.getSpan(axis);
        var obj0Min : Float = obj0.min;
        var obj0Max : Float = obj0.max;
        var obj1Min : Float = obj1.min;
        var obj1Max : Float = obj1.max;
        var tiny : Float = JMath3D.NUM_TINY;
        
        if (obj0Min > (obj1Max + JConfig.collToll + tiny) || obj1Min > (obj0Max + JConfig.collToll + tiny)) 
        {
            out.flag = true;
            return true;
        }
        if ((obj0Max > obj1Max) && (obj1Min > obj0Min)) 
        {
            out.depth = Math.min(obj0Max - obj1Min, obj1Max - obj0Min);
        }
        else if ((obj1Max > obj0Max) && (obj0Min > obj1Min)) 
        {
            out.depth = Math.min(obj1Max - obj0Min, obj0Max - obj1Min);
        }
        else 
        {
            out.depth = Math.min(obj0Max, obj1Max);
            out.depth -= Math.max(obj0Min, obj1Min);
        }
        out.flag = false;
        return false;
    }
    
    private function addPoint(contactPoints : Array<Vector3D>, pt : Vector3D, combinationDistanceSq : Float) : Bool
    {
        for (contactPoint in contactPoints)
        {
            if (contactPoint.subtract(pt).lengthSquared < combinationDistanceSq) 
            {
                contactPoint = JNumber3D.getScaleVector(contactPoint.add(pt), 0.5);
                return false;
            }
        }
        contactPoints.push(pt);
        return true;
    }
    
    private function getSupportPoint(box : JBox, axis : Vector3D) : Vector3D{
        var orientationCol : Array<Vector3D> = box.currentState.getOrientationCols();
        var _as : Float = axis.dotProduct(orientationCol[0]);
        var _au : Float = axis.dotProduct(orientationCol[1]);
        var _ad : Float = axis.dotProduct(orientationCol[2]);
        var tiny : Float = JMath3D.NUM_TINY;
        
        var p : Vector3D = box.currentState.position.clone();
        
        if (_as < -tiny) {
            p = p.add(JNumber3D.getScaleVector(orientationCol[0], 0.5 * box.sideLengths.x));
        }
        else if (_as >= tiny) {
            p = p.subtract(JNumber3D.getScaleVector(orientationCol[0], 0.5 * box.sideLengths.x));
        }
        
        if (_au < -tiny) {
            p = p.add(JNumber3D.getScaleVector(orientationCol[1], 0.5 * box.sideLengths.y));
        }
        else if (_au > tiny) {
            p = p.subtract(JNumber3D.getScaleVector(orientationCol[1], 0.5 * box.sideLengths.y));
        }
        
        if (_ad < -tiny) {
            p = p.add(JNumber3D.getScaleVector(orientationCol[2], 0.5 * box.sideLengths.z));
        }
        else if (_ad > tiny) {
            p = p.subtract(JNumber3D.getScaleVector(orientationCol[2], 0.5 * box.sideLengths.z));
        }
        return p;
    }
    
    private function getAABox2EdgeIntersectionPoints(contactPoint : Array<Vector3D>, origBoxSides : Vector3D, origBoxState : PhysicsState, edgePt0 : Vector3D, edgePt1 : Vector3D) : Int{
        var jDir : Int;
        var kDir : Int;
        var num : Int = 0;
        var iDir : Int;
        var iFace : Int;
        var dist0 : Float;
        var dist1 : Float;
        var frac : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        var pt : Vector3D;
        var edgeDir : Vector3D;
        
        edgeDir = edgePt1.subtract(edgePt0);
        edgeDir.normalize();
        var ptArr : Array<Float>;
        var faceOffsets : Array<Float>;
        var edgePt0Arr : Array<Float>;
        var edgePt1Arr : Array<Float>;
        var edgeDirArr : Array<Float>;
        var sidesArr : Array<Float>;
        edgePt0Arr = JNumber3D.toArray(edgePt0);
        edgePt1Arr = JNumber3D.toArray(edgePt1);
        edgeDirArr = JNumber3D.toArray(edgeDir);
        sidesArr = JNumber3D.toArray(JNumber3D.getScaleVector(origBoxSides, 0.5));
        iDir = 2;
        while (iDir >= 0){
            if (Math.abs(edgeDirArr[iDir]) < 0.1) {
                {iDir--;continue;
                }
            }
            jDir = (iDir + 1) % 3;
            kDir = (iDir + 2) % 3;
            faceOffsets = [-sidesArr[iDir], sidesArr[iDir]];
            iFace = 1;
            while (iFace >= 0){
                dist0 = edgePt0Arr[iDir] - faceOffsets[iFace];
                dist1 = edgePt1Arr[iDir] - faceOffsets[iFace];
                frac = -1;
                if (dist0 * dist1 < -tiny) {
                    frac = -dist0 / (dist1 - dist0);
                }
                else if (Math.abs(dist0) < tiny) {
                    frac = 0;
                }
                else if (Math.abs(dist1) < tiny) {
                    frac = 1;
                }
                if (frac >= 0) {
                    pt = JNumber3D.getScaleVector(edgePt0, 1 - frac).add(JNumber3D.getScaleVector(edgePt1, frac));
                    ptArr = JNumber3D.toArray(pt);
                    if ((ptArr[jDir] > -sidesArr[jDir] - tiny) && (ptArr[jDir] < sidesArr[jDir] + tiny) && (ptArr[kDir] > -sidesArr[kDir] - tiny) && (ptArr[kDir] < sidesArr[kDir] + tiny)) {
                        pt = origBoxState.orientation.transformVector(pt);
                        pt = pt.add(origBoxState.position);
                        addPoint(contactPoint, pt, combinationDist);
                        if (++num == 2) {
                            return num;
                        }
                    }
                }
                iFace--;
            }
            iDir--;
        }
        return num;
    }
    
    private function getBox2BoxEdgesIntersectionPoints(contactPoint : Array<Vector3D>, box0 : JBox, box1 : JBox, newState : Bool) : Float
    {
        var num : Float = 0;
        var seg : JSegment;
        var box0State : PhysicsState = (newState) ? box0.currentState : box0.oldState;
        var box1State : PhysicsState = (newState) ? box1.currentState : box1.oldState;
        var boxPts : Array<Vector3D> = box1.getCornerPointsInBoxSpace(box1State, box0State);
        
        var boxEdges : Array<EdgeData> = box1.edges;
        var edgePt0 : Vector3D;
        var edgePt1 : Vector3D;
        for (boxEdge in boxEdges)
        {
            edgePt0 = boxPts[boxEdge.ind0];
            edgePt1 = boxPts[boxEdge.ind1];
            num += getAABox2EdgeIntersectionPoints(contactPoint, box0.sideLengths, box0State, edgePt0, edgePt1);
            if (num >= 8) {
                return num;
            }
        }
        return num;
    }
    
    private function getBoxBoxIntersectionPoints(contactPoint : Array<Vector3D>, box0 : JBox, box1 : JBox, newState : Bool) : Int
    {
        getBox2BoxEdgesIntersectionPoints(contactPoint, box0, box1, newState);
        getBox2BoxEdgesIntersectionPoints(contactPoint, box1, box0, newState);
        return contactPoint.length;
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var box0 : JBox = try cast(info.body0, JBox) catch(e:Dynamic) null;
        var box1 : JBox = try cast(info.body1, JBox) catch(e:Dynamic) null;
        
        if (!box0.hitTestObject3D(box1)) 
            return;
        
        if (!box0.boundingBox.overlapTest(box1.boundingBox)) 
            return;
        
        var numTiny : Float = JMath3D.NUM_TINY;
        var numHuge : Float = JMath3D.NUM_HUGE;
        
        var dirs0Arr : Array<Vector3D> = box0.currentState.getOrientationCols();
        var dirs1Arr : Array<Vector3D> = box1.currentState.getOrientationCols();
        
        // the 15 potential separating axes
        var axes : Array<Vector3D> = [dirs0Arr[0], dirs0Arr[1], dirs0Arr[2], 
                dirs1Arr[0], dirs1Arr[1], dirs1Arr[2], 
                dirs0Arr[0].crossProduct(dirs1Arr[0]), 
                dirs0Arr[1].crossProduct(dirs1Arr[0]), 
                dirs0Arr[2].crossProduct(dirs1Arr[0]), 
                dirs0Arr[0].crossProduct(dirs1Arr[1]), 
                dirs0Arr[1].crossProduct(dirs1Arr[1]), 
                dirs0Arr[2].crossProduct(dirs1Arr[1]), 
                dirs0Arr[0].crossProduct(dirs1Arr[2]), 
                dirs0Arr[1].crossProduct(dirs1Arr[2]), 
                dirs0Arr[2].crossProduct(dirs1Arr[2])];
        
        var l2 : Float;
        // the overlap depths along each axis
        var overlapDepths : Array<SpanData> = new Array<SpanData>();
        var i : Int = 0;
        var axesLength : Int = axes.length;
        
        // see if the boxes are separate along any axis, and if not keep a
        // record of the depths along each axis
        var ax : Vector3D;
        for (i in 0...axesLength){
            overlapDepths.push( new SpanData() );
            
            l2 = axes[i].lengthSquared;
            if (l2 < numTiny) 
                continue;
            
            ax = axes[i].clone();
            ax.normalize();
            if (disjoint(overlapDepths[i], ax, box0, box1)) {
                info.body0.removeCollideBodies(info.body1);
                info.body1.removeCollideBodies(info.body0);
                return;
            }
        }
        
        // The box overlap, find the separation depth closest to 0.  
        var minDepth : Float = numHuge;
        var minAxis : Int = -1;
        axesLength = axes.length;
        for (i in 0...axesLength){
            l2 = axes[i].lengthSquared;
            if (l2 < numTiny) 
                {continue;
            }
            
            // If this axis is the minimum, select it
            if (overlapDepths[i].depth < minDepth) 
            {
                minDepth = overlapDepths[i].depth;
                minAxis = Std.int(i);
            }
        }
        
        if (minAxis == -1) {
            info.body0.removeCollideBodies(info.body1);
            info.body1.removeCollideBodies(info.body0);
            return;
        }
        
        // Make sure the axis is facing towards the box0. if not, invert it  
        var N : Vector3D = axes[minAxis].clone();
        if (box1.currentState.position.subtract(box0.currentState.position).dotProduct(N) > 0) 
            N.negate();
        
        var contactPointsFromOld : Bool = true;
        var contactPoints : Array<Vector3D> = new Array<Vector3D>();
        combinationDist = 0.05 * Math.min(Math.min(box0.sideLengths.x, Math.min(box0.sideLengths.y, box0.sideLengths.z)), Math.min(box1.sideLengths.x, Math.min(box1.sideLengths.y, box1.sideLengths.z) ));
        combinationDist += (JConfig.collToll * 3.464);
        combinationDist *= combinationDist;
        
        if (minDepth > -numTiny) 
            getBoxBoxIntersectionPoints(contactPoints, box0, box1, false);
        
        if (contactPoints.length == 0) 
        {
            contactPointsFromOld = false;
            getBoxBoxIntersectionPoints(contactPoints, box0, box1, true);
        }
        
        var bodyDelta : Vector3D = box0.currentState.position.subtract(box0.oldState.position).subtract(box1.currentState.position.subtract(box1.oldState.position));
        var bodyDeltaLen : Float = bodyDelta.dotProduct(N);
        var oldDepth : Float = minDepth + bodyDeltaLen;
        
        var SATPoint : Vector3D = new Vector3D();
        switch (minAxis)
        {
            //-----------------------------------------------------------------
            // Box0 face, Box1 Corner collision
            //-----------------------------------------------------------------
            case 0, 1, 2:
            {
                //-----------------------------------------------------------------
                // Get the lowest point on the box1 along box1 normal
                //-----------------------------------------------------------------
                SATPoint = getSupportPoint(box1, JNumber3D.getScaleVector(N, -1));
            }
            //-----------------------------------------------------------------
            // We have a Box2 corner/Box1 face collision
            //-----------------------------------------------------------------
            case 3, 4, 5:
            {
                //-----------------------------------------------------------------
                // Find with vertex on the triangle collided
                //-----------------------------------------------------------------
                SATPoint = getSupportPoint(box0, N);
            }
            //-----------------------------------------------------------------
            // We have an edge/edge colliiosn
            //-----------------------------------------------------------------
            case 6, 7, 8, 9, 10, 11, 12, 13, 14:
            {
                //-----------------------------------------------------------------
                // Retrieve which edges collided.
                //-----------------------------------------------------------------
                i = minAxis - 6;
                var ia : Int = Std.int(i / 3);
                var ib : Int = i - ia * 3;
                //-----------------------------------------------------------------
                // find two P0, P1 point on both edges.
                //-----------------------------------------------------------------
                var P0 : Vector3D = getSupportPoint(box0, N);
                var P1 : Vector3D = getSupportPoint(box1, JNumber3D.getScaleVector(N, -1));
                
                //-----------------------------------------------------------------
                // Find the edge intersection.
                //-----------------------------------------------------------------
                
                //-----------------------------------------------------------------
                // plane along N and F, and passing through PB
                //-----------------------------------------------------------------
                var planeNormal : Vector3D = N.crossProduct(dirs1Arr[ib]);
                var planeD : Float = planeNormal.dotProduct(P1);
                
                //-----------------------------------------------------------------
                // find the intersection t, where Pintersection = P0 + t*box edge dir
                //-----------------------------------------------------------------
                var div : Float = dirs0Arr[ia].dotProduct(planeNormal);
                
                //-----------------------------------------------------------------
                // plane and ray colinear, skip the intersection.
                //-----------------------------------------------------------------
                if (Math.abs(div) < numTiny) 
                    return;
                
                var t : Float = (planeD - P0.dotProduct(planeNormal)) / div;
                
                //-----------------------------------------------------------------
                // point on edge of box0
                //-----------------------------------------------------------------
                P0 = P0.add(JNumber3D.getScaleVector(dirs0Arr[ia], t));
                SATPoint = P0.add(JNumber3D.getScaleVector(N, 0.5 * minDepth));
            }
        }
        
        var collPts : Array<CollPointInfo>;
        if (contactPoints.length > 0) 
        {
            collPts = [for (i in 0...contactPoints.length) null];
            
            var minDist : Float = numHuge;
            var maxDist : Float = -numHuge;
            var dist : Float;
            var depth : Float;
            var depthScale : Float;
            
            var cpInfo : CollPointInfo;
            var contactPoint : Vector3D;
            
            for (contactPoint in contactPoints)
            {
                dist = contactPoint.subtract(SATPoint).length;
                
                if (dist < minDist) 
                    minDist = dist;
                
                if (dist > maxDist) 
                    maxDist = dist;
            }
            
            if (maxDist < minDist + numTiny) 
                maxDist = minDist + numTiny;
            
            i = 0;
            for (contactPoint in contactPoints)
            {
                dist = contactPoint.subtract(SATPoint).length;
                depthScale = (dist - minDist) / (maxDist - minDist);
                depth = (1 - depthScale) * oldDepth;
                cpInfo = new CollPointInfo();
                
                if (contactPointsFromOld) 
                {
                    cpInfo.r0 = contactPoint.subtract(box0.oldState.position);
                    cpInfo.r1 = contactPoint.subtract(box1.oldState.position);
                }
                else 
                {
                    cpInfo.r0 = contactPoint.subtract(box0.currentState.position);
                    cpInfo.r1 = contactPoint.subtract(box1.currentState.position);
                }
                
                cpInfo.initialPenetration = depth;
                collPts[i++] = cpInfo;
            }
        }
        else 
        {
            var cpInfo = new CollPointInfo();
            cpInfo.r0 = SATPoint.subtract(box0.currentState.position);
            cpInfo.r1 = SATPoint.subtract(box1.currentState.position);
            cpInfo.initialPenetration = oldDepth;
            
            collPts = new Array<CollPointInfo>();
            collPts.push( cpInfo );
        }
        
        var collInfo : CollisionInfo = new CollisionInfo();
        collInfo.objInfo = info;
        collInfo.dirToBody = N;
        collInfo.pointInfo = collPts;
        
        var mat : MaterialProperties = new MaterialProperties();
        mat.restitution = 0.5 * (box0.material.restitution + box1.material.restitution);
        mat.friction = 0.5 * (box0.material.friction + box1.material.friction);
        collInfo.mat = mat;
        collArr.push(collInfo);
        info.body0.collisions.push(collInfo);
        info.body1.collisions.push(collInfo);
        info.body0.addCollideBody(info.body1);
        info.body1.addCollideBody(info.body0);
    }
}
