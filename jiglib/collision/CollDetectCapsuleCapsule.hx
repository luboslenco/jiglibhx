package jiglib.collision;

import jiglib.collision.CollDetectFunctor;
import jiglib.collision.CollDetectInfo;
import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;

class CollDetectCapsuleCapsule extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "CapsuleCapsule";
        type0 = "CAPSULE";
        type1 = "CAPSULE";
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var capsule0 : JCapsule = try cast(info.body0, JCapsule) catch(e:Dynamic) null;
        var capsule1 : JCapsule = try cast(info.body1, JCapsule) catch(e:Dynamic) null;
        
        if (!capsule0.hitTestObject3D(capsule1)) 
        {
            return;
        }
        
        if (!capsule0.boundingBox.overlapTest(capsule1.boundingBox)) {
            return;
        }
        
        var collPts : Array<CollPointInfo> = new Array<CollPointInfo>();
        var cpInfo : CollPointInfo;
        
        var averageNormal : Vector3D = new Vector3D();
        var oldSeg0 : JSegment;
        var newSeg0 : JSegment;
        var oldSeg1 : JSegment;
        var newSeg1 : JSegment;
        oldSeg0 = new JSegment(capsule0.getEndPos(capsule0.oldState), JNumber3D.getScaleVector(capsule0.oldState.getOrientationCols()[1], -capsule0.length));
        newSeg0 = new JSegment(capsule0.getEndPos(capsule0.currentState), JNumber3D.getScaleVector(capsule0.currentState.getOrientationCols()[1], -capsule0.length));
        oldSeg1 = new JSegment(capsule1.getEndPos(capsule1.oldState), JNumber3D.getScaleVector(capsule1.oldState.getOrientationCols()[1], -capsule1.length));
        newSeg1 = new JSegment(capsule1.getEndPos(capsule1.currentState), JNumber3D.getScaleVector(capsule1.currentState.getOrientationCols()[1], -capsule1.length));
        
        var radSum : Float = capsule0.radius + capsule1.radius;
        
        var oldObj : Array<Float> = [0, 0];
        var oldDistSq : Float = oldSeg0.segmentSegmentDistanceSq(oldObj, oldSeg1);
        var newObj : Array<Float> = [0, 0];
        var newDistSq : Float = newSeg0.segmentSegmentDistanceSq(oldObj, newSeg1);
        
        if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JConfig.collToll, 2)) 
        {
            var pos0 : Vector3D = oldSeg0.getPoint(oldObj[0]);
            var pos1 : Vector3D = oldSeg1.getPoint(oldObj[1]);
            
            var delta : Vector3D = pos0.subtract(pos1);
            var dist : Float = Math.sqrt(oldDistSq);
            var depth : Float = radSum - dist;
            
            if (dist > JMath3D.NUM_TINY) 
            {
                delta = JNumber3D.getDivideVector(delta, dist);
            }
            else 
            {
                delta = JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(Vector3D.Y_AXIS);
            }
            
            var worldPos : Vector3D = pos1.add(JNumber3D.getScaleVector(delta, capsule1.radius - 0.5 * depth));
            averageNormal = averageNormal.add(delta);
            
            cpInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(capsule0.oldState.position);
            cpInfo.r1 = worldPos.subtract(capsule1.oldState.position);
            cpInfo.initialPenetration = depth;
            collPts.push( cpInfo );
        }
        
        if (collPts.length > 0) 
        {
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = averageNormal;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (capsule0.material.restitution + capsule1.material.restitution);
            mat.friction = 0.5 * (capsule0.material.friction + capsule1.material.friction);
            collInfo.mat = mat;
            collArr.push(collInfo);
            info.body0.collisions.push(collInfo);
            info.body1.collisions.push(collInfo);
            info.body0.addCollideBody(info.body1);
            info.body1.addCollideBody(info.body0);
        }
        else {
            info.body0.removeCollideBodies(info.body1);
            info.body1.removeCollideBodies(info.body0);
        }
    }
}


