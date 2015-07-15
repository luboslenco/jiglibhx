package jiglib.collision;

import jiglib.collision.CollDetectFunctor;
import jiglib.collision.CollDetectInfo;
import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;
import jiglib.physics.RigidBody;

class CollDetectCapsuleBox extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "CapsuleBox";
        type0 = "CAPSULE";
        type1 = "BOX";
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var tempBody : RigidBody;
        if (info.body0.type == "BOX") 
        {
            tempBody = info.body0;
            info.body0 = info.body1;
            info.body1 = tempBody;
        }
        
        var capsule : JCapsule = try cast(info.body0, JCapsule) catch(e:Dynamic) null;
        var box : JBox = try cast(info.body1, JBox) catch(e:Dynamic) null;
        
        if (!capsule.hitTestObject3D(box)) 
        {
            return;
        }
        if (!capsule.boundingBox.overlapTest(box.boundingBox)) {
            return;
        }
        
        var collPts : Array<CollPointInfo> = new Array<CollPointInfo>();
        var cpInfo : CollPointInfo;
        
        var averageNormal : Vector3D = new Vector3D();
        var oldSeg : JSegment = new JSegment(capsule.getBottomPos(capsule.oldState), JNumber3D.getScaleVector(capsule.oldState.getOrientationCols()[1], capsule.length));
        var newSeg : JSegment = new JSegment(capsule.getBottomPos(capsule.currentState), JNumber3D.getScaleVector(capsule.currentState.getOrientationCols()[1], capsule.length));
        var radius : Float = capsule.radius;
        
        var oldObj : Array<Float> = [0, 0, 0, 0];
        var oldDistSq : Float = oldSeg.segmentBoxDistanceSq(oldObj, box, box.oldState);
        var newObj : Array<Float> = [0, 0, 0, 0];
        var newDistSq : Float = newSeg.segmentBoxDistanceSq(newObj, box, box.currentState);
        var arr : Array<Vector3D> = box.oldState.getOrientationCols();
        
        if (Math.min(oldDistSq, newDistSq) < Math.pow(radius + JConfig.collToll, 2)) 
        {
            var segPos : Vector3D = oldSeg.getPoint(oldObj[3]);
            var boxPos : Vector3D = box.oldState.position.clone();
            boxPos = boxPos.add(JNumber3D.getScaleVector(arr[0], oldObj[0]));
            boxPos = boxPos.add(JNumber3D.getScaleVector(arr[1], oldObj[1]));
            boxPos = boxPos.add(JNumber3D.getScaleVector(arr[2], oldObj[2]));
            
            var dist : Float = Math.sqrt(oldDistSq);
            var depth : Float = radius - dist;
            
            var dir : Vector3D;
            if (dist > JMath3D.NUM_TINY) 
            {
                dir = segPos.subtract(boxPos);
                dir.normalize();
            }
            else if (segPos.subtract(box.oldState.position).length > JMath3D.NUM_TINY) 
            {
                dir = segPos.subtract(box.oldState.position);
                dir.normalize();
            }
            else 
            {
                dir = JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(Vector3D.Y_AXIS);
            }
            averageNormal = averageNormal.add(dir);
            
            cpInfo = new CollPointInfo();
            cpInfo.r0 = boxPos.subtract(capsule.oldState.position);
            cpInfo.r1 = boxPos.subtract(box.oldState.position);
            cpInfo.initialPenetration = depth;
            collPts[0] = cpInfo;
        }
        
        if (collPts.length > 0) 
        {
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = averageNormal;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (capsule.material.restitution + box.material.restitution);
            mat.friction = 0.5 * (capsule.material.friction + box.material.friction);
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

