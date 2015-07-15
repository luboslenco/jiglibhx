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

class CollDetectCapsulePlane extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "CapsulePlane";
        type0 = "CAPSULE";
        type1 = "PLANE";
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var tempBody : RigidBody;
        if (info.body0.type == "PLANE") 
        {
            tempBody = info.body0;
            info.body0 = info.body1;
            info.body1 = tempBody;
        }
        
        var capsule : JCapsule = try cast(info.body0, JCapsule) catch(e:Dynamic) null;
        var plane : JPlane = try cast(info.body1, JPlane) catch(e:Dynamic) null;
        
        var collPts : Array<CollPointInfo> = new Array<CollPointInfo>();
        var cpInfo : CollPointInfo;
        
        var oldPos : Vector3D = capsule.getBottomPos(capsule.oldState);
        var oldDist : Float = plane.pointPlaneDistance(oldPos);
        var newPos : Vector3D = capsule.getBottomPos(capsule.currentState);
        var newDist : Float = plane.pointPlaneDistance(newPos);
        
        if (Math.min(oldDist, newDist) < capsule.radius + JConfig.collToll) 
        {
            var oldDepth : Float = capsule.radius - oldDist;
            var worldPos : Vector3D = oldPos.subtract(JNumber3D.getScaleVector(plane.normal, capsule.radius));
            
            cpInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(capsule.oldState.position);
            cpInfo.r1 = worldPos.subtract(plane.oldState.position);
            cpInfo.initialPenetration = oldDepth;
            collPts.push(cpInfo);
        }
        
        oldPos = capsule.getEndPos(capsule.oldState);
        newPos = capsule.getEndPos(capsule.currentState);
        oldDist = plane.pointPlaneDistance(oldPos);
        newDist = plane.pointPlaneDistance(newPos);
        if (Math.min(oldDist, newDist) < capsule.radius + JConfig.collToll) 
        {
            var oldDepth:Float = capsule.radius - oldDist;
            var worldPos : Vector3D = oldPos.subtract(JNumber3D.getScaleVector(plane.normal, capsule.radius));
            
            cpInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(capsule.oldState.position);
            cpInfo.r1 = worldPos.subtract(plane.oldState.position);
            cpInfo.initialPenetration = oldDepth;
            collPts.push(cpInfo);
        }
        
        if (collPts.length > 0) 
        {
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = plane.normal.clone();
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (capsule.material.restitution + plane.material.restitution);
            mat.friction = 0.5 * (capsule.material.friction + plane.material.friction);
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


