package jiglib.collision;

import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;
import jiglib.physics.RigidBody;

class CollDetectSphereBox extends CollDetectFunctor
{
    public function new()
    {
        super();
        name = "SphereBox";
        type0 = "SPHERE";
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
        
        var sphere : JSphere = try cast(info.body0, JSphere) catch(e:Dynamic) null;
        var box : JBox = try cast(info.body1, JBox) catch(e:Dynamic) null;
        
        if (!sphere.hitTestObject3D(box)) 
        {
            return;
        }
        if (!sphere.boundingBox.overlapTest(box.boundingBox)) 
        {
            return;
        }
        
        //var boxPos:Vector3D = box.oldState.position;
        //var spherePos:Vector3D = sphere.oldState.position;  
        
        var oldBoxPoint : Array<Vector3D> = [new Vector3D()];
        var newBoxPoint : Array<Vector3D> = [new Vector3D()];
        
        var oldDist : Float;
        var newDist : Float;
        var oldDepth : Float;
        var newDepth : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        oldDist = box.getDistanceToPoint(box.oldState, oldBoxPoint, sphere.oldState.position);
        newDist = box.getDistanceToPoint(box.currentState, newBoxPoint, sphere.currentState.position);
        
        var _oldBoxPosition : Vector3D = oldBoxPoint[0];
        
        oldDepth = sphere.radius - oldDist;
        newDepth = sphere.radius - newDist;
        if (Math.max(oldDepth, newDepth) > -JConfig.collToll) 
        {
            var dir : Vector3D;
            var collPts : Array<CollPointInfo> = [null];
            if (oldDist < -tiny) 
            {
                dir = _oldBoxPosition.subtract(sphere.oldState.position).subtract(_oldBoxPosition);
                dir.normalize();
            }
            else if (oldDist > tiny) 
            {
                dir = sphere.oldState.position.subtract(_oldBoxPosition);
                dir.normalize();
            }
            else 
            {
                dir = sphere.oldState.position.subtract(box.oldState.position);
                dir.normalize();
            }
            
            var cpInfo : CollPointInfo = new CollPointInfo();
            cpInfo.r0 = _oldBoxPosition.subtract(sphere.oldState.position);
            cpInfo.r1 = _oldBoxPosition.subtract(box.oldState.position);
            cpInfo.initialPenetration = oldDepth;
            collPts[0] = cpInfo;
            
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = dir;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (sphere.material.restitution + box.material.restitution);
            mat.friction = 0.5 * (sphere.material.friction + box.material.friction);
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
