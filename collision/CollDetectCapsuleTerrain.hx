package jiglib.collision;

import jiglib.collision.CollDetectFunctor;
import jiglib.collision.CollDetectInfo;
import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.data.TerrainData;
import jiglib.geometry.JCapsule;
import jiglib.geometry.JTerrain;
import jiglib.math.JNumber3D;
import jiglib.math.Vector3D;
import jiglib.physics.MaterialProperties;
import jiglib.physics.RigidBody;

class CollDetectCapsuleTerrain extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "BoxTerrain";
        type0 = "CAPSULE";
        type1 = "TERRAIN";
    }
    
    override public function collDetect(info : CollDetectInfo, collArr : Array<CollisionInfo>) : Void
    {
        var tempBody : RigidBody;
        if (info.body0.type == "TERRAIN") 
        {
            tempBody = info.body0;
            info.body0 = info.body1;
            info.body1 = tempBody;
        }
        
        var capsule : JCapsule = try cast(info.body0, JCapsule) catch(e:Dynamic) null;
        var terrain : JTerrain = try cast(info.body1, JTerrain) catch(e:Dynamic) null;
        
        var collPts : Array<CollPointInfo> = new Array<CollPointInfo>();
        var cpInfo : CollPointInfo;
        
        var averageNormal : Vector3D = new Vector3D();
        var pos1 : Vector3D = capsule.getBottomPos(capsule.oldState);
        var pos2 : Vector3D = capsule.getBottomPos(capsule.currentState);
        var obj1 : TerrainData = terrain.getHeightAndNormalByPoint(pos1);
        var obj2 : TerrainData = terrain.getHeightAndNormalByPoint(pos2);
        if (Math.min(obj1.height, obj2.height) < JConfig.collToll + capsule.radius) {
            var oldDepth : Float = capsule.radius - obj1.height;
            var worldPos : Vector3D = pos1.subtract(JNumber3D.getScaleVector(obj2.normal, capsule.radius));
            cpInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(capsule.oldState.position);
            cpInfo.r1 = worldPos.subtract(terrain.oldState.position);
            cpInfo.initialPenetration = oldDepth;
            collPts.push(cpInfo);
            averageNormal = averageNormal.add(obj2.normal);
        }
        
        pos1 = capsule.getEndPos(capsule.oldState);
        pos2 = capsule.getEndPos(capsule.currentState);
        obj1 = terrain.getHeightAndNormalByPoint(pos1);
        obj2 = terrain.getHeightAndNormalByPoint(pos2);
        if (Math.min(obj1.height, obj2.height) < JConfig.collToll + capsule.radius) {
            var oldDepth:Float = capsule.radius - obj1.height;
            var worldPos : Vector3D = pos1.subtract(JNumber3D.getScaleVector(obj2.normal, capsule.radius));
            cpInfo = new CollPointInfo();
            cpInfo.r0 = worldPos.subtract(capsule.oldState.position);
            cpInfo.r1 = worldPos.subtract(terrain.oldState.position);
            cpInfo.initialPenetration = oldDepth;
            collPts.push(cpInfo);
            averageNormal = averageNormal.add(obj2.normal);
        }
        
        if (collPts.length > 0) 
        {
            averageNormal.normalize();
            
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = averageNormal;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (capsule.material.restitution + terrain.material.restitution);
            mat.friction = 0.5 * (capsule.material.friction + terrain.material.friction);
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

