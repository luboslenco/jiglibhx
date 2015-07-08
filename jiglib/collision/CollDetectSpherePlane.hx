package jiglib.collision;

import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.MaterialProperties;
import jiglib.physics.RigidBody;

class CollDetectSpherePlane extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "SpherePlane";
        type0 = "SPHERE";
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
        
        var sphere : JSphere = try cast(info.body0, JSphere) catch(e:Dynamic) null;
        var plane : JPlane = try cast(info.body1, JPlane) catch(e:Dynamic) null;
        
        var oldDist : Float;
        var newDist : Float;
        var depth : Float;
        oldDist = plane.pointPlaneDistance(sphere.oldState.position);
        newDist = plane.pointPlaneDistance(sphere.currentState.position);
        
        if (Math.min(newDist, oldDist) > sphere.boundingSphere + JConfig.collToll) 
        {
            info.body0.removeCollideBodies(info.body1);
            info.body1.removeCollideBodies(info.body0);
            return;
        }
        
        var collPts : Array<CollPointInfo> = [null];
        var cpInfo : CollPointInfo;
        depth = sphere.radius - oldDist;
        
        var worldPos : Vector3D = sphere.oldState.position.subtract(JNumber3D.getScaleVector(plane.normal, sphere.radius));
        cpInfo = new CollPointInfo();
        cpInfo.r0 = worldPos.subtract(sphere.oldState.position);
        cpInfo.r1 = worldPos.subtract(plane.oldState.position);
        cpInfo.initialPenetration = depth;
        collPts[0] = cpInfo;
        
        var collInfo : CollisionInfo = new CollisionInfo();
        collInfo.objInfo = info;
        collInfo.dirToBody = plane.normal.clone();
        collInfo.pointInfo = collPts;
        
        var mat : MaterialProperties = new MaterialProperties();
        mat.restitution = 0.5 * (sphere.material.restitution + plane.material.restitution);
        mat.friction = 0.5 * (sphere.material.friction + plane.material.friction);
        collInfo.mat = mat;
        collArr.push(collInfo);
        info.body0.collisions.push(collInfo);
        info.body1.collisions.push(collInfo);
        info.body0.addCollideBody(info.body1);
        info.body1.addCollideBody(info.body0);
    }
}
