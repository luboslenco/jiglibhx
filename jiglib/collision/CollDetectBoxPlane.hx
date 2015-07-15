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

class CollDetectBoxPlane extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "BoxPlane";
        type0 = "BOX";
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
        
        var box : JBox = try cast(info.body0, JBox) catch(e:Dynamic) null;
        var plane : JPlane = try cast(info.body1, JPlane) catch(e:Dynamic) null;
        
        var centreDist : Float = plane.pointPlaneDistance(box.currentState.position);
        if (centreDist > box.boundingSphere + JConfig.collToll) 
            return;
        
        var newPts : Array<Vector3D> = box.getCornerPoints(box.currentState);
        var oldPts : Array<Vector3D> = box.getCornerPoints(box.oldState);
        var collPts : Array<CollPointInfo> = new Array<CollPointInfo>();
        var cpInfo : CollPointInfo;
        var newPt : Vector3D;
        var oldPt : Vector3D;
        var newDepth : Float;
        var oldDepth : Float;
        var newPts_length : Int = newPts.length;
        
        for (i in 0...newPts_length){
            newPt = newPts[i];
            oldPt = oldPts[i];
            newDepth = -1 * plane.pointPlaneDistance(newPt);
            oldDepth = -1 * plane.pointPlaneDistance(oldPt);
            
            if (Math.max(newDepth, oldDepth) > -JConfig.collToll) 
            {
                cpInfo = new CollPointInfo();
                cpInfo.r0 = oldPt.subtract(box.oldState.position);
                cpInfo.r1 = oldPt.subtract(plane.oldState.position);
                cpInfo.initialPenetration = oldDepth;
                collPts.push(cpInfo);
            }
        }
        
        if (collPts.length > 0) 
        {
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = plane.normal.clone();
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (box.material.restitution + plane.material.restitution);
            mat.friction = 0.5 * (box.material.friction + plane.material.friction);
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

