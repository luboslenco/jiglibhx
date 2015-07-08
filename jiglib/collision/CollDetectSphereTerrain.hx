package jiglib.collision;

import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;

import jiglib.cof.JConfig;
import jiglib.data.TerrainData;
import jiglib.geometry.JSphere;
import jiglib.geometry.JTerrain;
import jiglib.math.Vector3D;
import jiglib.math.JNumber3D;
import jiglib.physics.MaterialProperties;
import jiglib.physics.RigidBody;

class CollDetectSphereTerrain extends CollDetectFunctor
{
    
    public function new()
    {
        super();
        name = "SphereTerrain";
        type0 = "SPHERE";
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
        
        var sphere : JSphere = try cast(info.body0, JSphere) catch(e:Dynamic) null;
        var terrain : JTerrain = try cast(info.body1, JTerrain) catch(e:Dynamic) null;
        
        var obj : TerrainData = terrain.getHeightAndNormalByPoint(sphere.currentState.position);
        if (obj.height < JConfig.collToll + sphere.radius) {
            var dist : Float = terrain.getHeightByPoint(sphere.oldState.position);
            var depth : Float = sphere.radius - dist;
            
            var Pt : Vector3D = sphere.oldState.position.subtract(JNumber3D.getScaleVector(obj.normal, sphere.radius));
            
            var collPts : Array<CollPointInfo> = [null];
            var cpInfo : CollPointInfo = new CollPointInfo();
            cpInfo.r0 = Pt.subtract(sphere.oldState.position);
            cpInfo.r1 = Pt.subtract(terrain.oldState.position);
            cpInfo.initialPenetration = depth;
            collPts[0] = cpInfo;
            
            var collInfo : CollisionInfo = new CollisionInfo();
            collInfo.objInfo = info;
            collInfo.dirToBody = obj.normal;
            collInfo.pointInfo = collPts;
            
            var mat : MaterialProperties = new MaterialProperties();
            mat.restitution = 0.5 * (sphere.material.restitution + terrain.material.restitution);
            mat.friction = 0.5 * (sphere.material.friction + terrain.material.friction);
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

