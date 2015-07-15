package jiglib.collision;

import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;

class CollDetectSphereSphere extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "SphereSphere";
        type0 = "SPHERE";
        type1 = "SPHERE";
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var sphere0 : JSphere = try cast(info.body0, JSphere) catch(e:Dynamic) null;
        var sphere1 : JSphere = try cast(info.body1, JSphere) catch(e:Dynamic) null;
        
        var oldDelta : Vector3D = sphere0.oldState.position.subtract(sphere1.oldState.position);
        var newDelta : Vector3D = sphere0.currentState.position.subtract(sphere1.currentState.position);
        
        var oldDistSq : Float;
        var newDistSq : Float;
        var radSum : Float;
        var oldDist : Float;
        var depth : Float;
        oldDistSq = oldDelta.lengthSquared;
        newDistSq = newDelta.lengthSquared;
        radSum = sphere0.radius + sphere1.radius;
        
        if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JConfig.collToll, 2)) 
        {
            oldDist = Math.sqrt(oldDistSq);
            depth = radSum - oldDist;
            if (oldDist > JMath3D.NUM_TINY) 
            {
                oldDelta = JNumber3D.getDivideVector(oldDelta, oldDist);
            }
            else 
            {
                oldDelta = JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(Vector3D.Y_AXIS);
            }
            
            var worldPos : Vector3D = sphere1.oldState.position.add(JNumber3D.getScaleVector(oldDelta, sphere1.radius - 0.5 * depth));
            
            var collPts : Array<CollPointInfo> = [null];
            var cpInfo : CollPointInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(sphere0.oldState.position);
            cpInfo.r1 = worldPos.subtract(sphere1.oldState.position);
            cpInfo.initialPenetration = depth;
            collPts[0] = cpInfo;
            
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = oldDelta;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (sphere0.material.restitution + sphere1.material.restitution);
            mat.friction = 0.5 * (sphere0.material.friction + sphere1.material.friction);
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
