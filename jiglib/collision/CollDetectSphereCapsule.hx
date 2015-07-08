package jiglib.collision;

import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;
import jiglib.physics.RigidBody;

class CollDetectSphereCapsule extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "SphereCapsule";
        type0 = "SPHERE";
        type1 = "CAPSULE";
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var tempBody : RigidBody;
        if (info.body0.type == "CAPSULE") 
        {
            tempBody = info.body0;
            info.body0 = info.body1;
            info.body1 = tempBody;
        }
        
        var sphere : JSphere = try cast(info.body0, JSphere) catch(e:Dynamic) null;
        var capsule : JCapsule = try cast(info.body1, JCapsule) catch(e:Dynamic) null;
        
        if (!sphere.hitTestObject3D(capsule)) 
        {
            return;
        }
        
        if (!sphere.boundingBox.overlapTest(capsule.boundingBox)) {
            return;
        }
        
        var oldSeg : JSegment = new JSegment(capsule.getBottomPos(capsule.oldState), JNumber3D.getScaleVector(capsule.oldState.getOrientationCols()[1], capsule.length));
        var newSeg : JSegment = new JSegment(capsule.getBottomPos(capsule.currentState), JNumber3D.getScaleVector(capsule.currentState.getOrientationCols()[1], capsule.length));
        var radSum : Float = sphere.radius + capsule.radius;
        
        var oldObj : Array<Float> = [0];
        var oldDistSq : Float = oldSeg.pointSegmentDistanceSq(oldObj, sphere.oldState.position);
        var newObj : Array<Float> = [0];
        var newDistSq : Float = newSeg.pointSegmentDistanceSq(newObj, sphere.currentState.position);
        
        if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JConfig.collToll, 2)) 
        {
            var segPos : Vector3D = oldSeg.getPoint(oldObj[0]);
            var delta : Vector3D = sphere.oldState.position.subtract(segPos);
            
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
            
            var worldPos : Vector3D = segPos.add(JNumber3D.getScaleVector(delta, capsule.radius - 0.5 * depth));
            
            var collPts : Array<CollPointInfo> = [null];
            var cpInfo : CollPointInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(sphere.oldState.position);
            cpInfo.r1 = worldPos.subtract(capsule.oldState.position);
            cpInfo.initialPenetration = depth;
            collPts[0] = cpInfo;
            
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = delta;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (sphere.material.restitution + capsule.material.restitution);
            mat.friction = 0.5 * (sphere.material.friction + capsule.material.friction);
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


