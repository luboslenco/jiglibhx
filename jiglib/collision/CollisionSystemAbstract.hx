package jiglib.collision;

import jiglib.data.CollOutBodyData;
import jiglib.geometry.JSegment;
import jiglib.math.JNumber3D;
import jiglib.math.JMath3D;
import jiglib.math.Vector3D;
import jiglib.physics.RigidBody;

class CollisionSystemAbstract
{
    public var numCollisionsChecks(get, never) : Int;

    public var detectionFunctors : Map<String, CollDetectFunctor>;
    public var collBody : Array<RigidBody>;
    public var _numCollisionsChecks : Int = 0;
    
    public var startPoint : Vector3D;  //for grid system  
    
    public function new()
    {
        collBody = new Array<RigidBody>();
        detectionFunctors = new Map();
        detectionFunctors.set("BOX_BOX", new CollDetectBoxBox());
        detectionFunctors.set("BOX_SPHERE", new CollDetectSphereBox());
        detectionFunctors.set("BOX_CAPSULE", new CollDetectCapsuleBox());
        detectionFunctors.set("BOX_PLANE", new CollDetectBoxPlane());
        detectionFunctors.set("BOX_TERRAIN", new CollDetectBoxTerrain());
        detectionFunctors.set("BOX_TRIANGLEMESH", new CollDetectBoxMesh());
        detectionFunctors.set("SPHERE_BOX", new CollDetectSphereBox());
        detectionFunctors.set("SPHERE_SPHERE", new CollDetectSphereSphere());
        detectionFunctors.set("SPHERE_CAPSULE", new CollDetectSphereCapsule());
        detectionFunctors.set("SPHERE_PLANE", new CollDetectSpherePlane());
        detectionFunctors.set("SPHERE_TERRAIN", new CollDetectSphereTerrain());
        detectionFunctors.set("SPHERE_TRIANGLEMESH", new CollDetectSphereMesh());
        detectionFunctors.set("CAPSULE_CAPSULE", new CollDetectCapsuleCapsule());
        detectionFunctors.set("CAPSULE_BOX", new CollDetectCapsuleBox());
        detectionFunctors.set("CAPSULE_SPHERE", new CollDetectSphereCapsule());
        detectionFunctors.set("CAPSULE_PLANE", new CollDetectCapsulePlane());
        detectionFunctors.set("CAPSULE_TERRAIN", new CollDetectCapsuleTerrain());
        detectionFunctors.set("PLANE_BOX", new CollDetectBoxPlane());
        detectionFunctors.set("PLANE_SPHERE", new CollDetectSpherePlane());
        detectionFunctors.set("PLANE_CAPSULE", new CollDetectCapsulePlane());
        detectionFunctors.set("TERRAIN_SPHERE", new CollDetectSphereTerrain());
        detectionFunctors.set("TERRAIN_BOX", new CollDetectBoxTerrain());
        detectionFunctors.set("TERRAIN_CAPSULE", new CollDetectCapsuleTerrain());
        detectionFunctors.set("TRIANGLEMESH_SPHERE", new CollDetectSphereMesh());
        detectionFunctors.set("TRIANGLEMESH_BOX", new CollDetectBoxMesh());
    }
    
    public function addCollisionBody(body : RigidBody) : Void
    {
        if (collBody.indexOf(body) < 0) 
            collBody.push(body);
    }
    
    public function removeCollisionBody(body : RigidBody) : Void
    {
        var idx = collBody.indexOf(body);
        if (idx >= 0) 
            collBody.splice(idx, 1);
    }
    
    public function removeAllCollisionBodies() : Void
    {
        //collBody.length = 0;
        collBody.splice(0, collBody.length);
    }
    
    // Detects collisions between the body and all the registered collision bodies
    public function detectCollisions(body : RigidBody, collArr : Array<CollisionInfo>) : Void
    {
        if (!body.isActive) 
            return;
        
        var info : CollDetectInfo;
        var fu : CollDetectFunctor;
        
        for (_collBody in collBody)
        {
            if (body == _collBody) 
            {
                continue;
            }
            if (checkCollidables(body, _collBody) && detectionFunctors.get(body.type + "_" + _collBody.type) != null) 
            {
                info = new CollDetectInfo();
                info.body0 = body;
                info.body1 = _collBody;
                fu = detectionFunctors.get(info.body0.type + "_" + info.body1.type);
                fu.collDetect(info, collArr);
            }
        }
    }
    
    // Detects collisions between the all bodies
    public function detectAllCollisions(bodies : Array<RigidBody>, collArr : Array<CollisionInfo>) : Void
    {
        
    }
    
    public function collisionSkinMoved(colBody : RigidBody) : Void
    {
        // used for grid
        
    }
    
    public function segmentIntersect(out : CollOutBodyData, seg : JSegment, ownerBody : RigidBody) : Bool
    {
        out.frac = JMath3D.NUM_HUGE;
        out.position = new Vector3D();
        out.normal = new Vector3D();
        
        var obj : CollOutBodyData = new CollOutBodyData();
        for (_collBody in collBody)
        {
            if (_collBody != ownerBody && segmentBounding(seg, _collBody)) 
            {
                if (_collBody.segmentIntersect(obj, seg, _collBody.currentState)) 
                {
                    if (obj.frac < out.frac) 
                    {
                        out.position = obj.position;
                        out.normal = obj.normal;
                        out.frac = obj.frac;
                        out.rigidBody = _collBody;
                    }
                }
            }
        }
        
        if (out.frac > 1) 
            return false;
        
        if (out.frac < 0) 
        {
            out.frac = 0;
        }
        else if (out.frac > 1) 
        {
            out.frac = 1;
        }
        
        return true;
    }
    
    public function segmentBounding(seg : JSegment, obj : RigidBody) : Bool
    {
        var pos : Vector3D = seg.getPoint(0.5);
        var r : Float = seg.delta.length / 2;
        
        var num1 : Float = pos.subtract(obj.currentState.position).length;
        var num2 : Float = r + obj.boundingSphere;
        
        if (num1 <= num2) 
            return true
        else 
        return false;
    }
    
    private function get_numCollisionsChecks() : Int
    {
        return _numCollisionsChecks;
    }
    
    private function checkCollidables(body0 : RigidBody, body1 : RigidBody) : Bool
    {
        if (body0.nonCollidables.length == 0 && body1.nonCollidables.length == 0) 
            return true;
        
        if (body0.nonCollidables.indexOf(body1) > -1) 
            return false;
        
        if (body1.nonCollidables.indexOf(body0) > -1) 
            return false;
        
        return true;
    }
}
