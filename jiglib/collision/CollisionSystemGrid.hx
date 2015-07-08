package jiglib.collision;

import jiglib.collision.CollisionSystemGridEntry;

import jiglib.data.CollOutBodyData;
import jiglib.geometry.JAABox;
import jiglib.geometry.JSegment;
import jiglib.math.JMath3D;
import jiglib.math.JNumber3D;
import jiglib.math.Vector3D;
import jiglib.physics.RigidBody;

private class GridData
{
    public var i:Int;
    public var j:Int;
    public var k:Int;
    public var fi:Float;
    public var fj:Float;
    public var fk:Float;

    public function new() { }
}

class CollisionSystemGrid extends CollisionSystemAbstract
{
    private var gridEntries : Array<CollisionSystemGridEntry>;
    
    private var overflowEntries : CollisionSystemGridEntry;
    
    private var nx : Int;
    private var ny : Int;
    private var nz : Int;
    
    private var dx : Float;
    private var dy : Float;
    private var dz : Float;
    
    private var sizeX : Float;
    private var sizeY : Float;
    private var sizeZ : Float;
    
    // minimum of the grid deltas
    private var minDelta : Float;
    
    /*
		* Initializes a new CollisionSystem which uses a grid to speed up collision detection.
		* Use this system for larger scenes with many objects.
		* @param sx start point of grid in X axis.
		* @param sy start point of grid in Y axis.
		* @param sz start point of grid in Z axis.
		* @param nx Number of GridEntries in X Direction.
		* @param ny Number of GridEntries in Y Direction.
		* @param nz Number of GridEntries in Z Direction.
		* @param dx Size of a single GridEntry in X Direction.
		* @param dy Size of a single GridEntry in Y Direction.
		* @param dz Size of a single GridEntry in Z Direction.
		*/
    public function new(sx : Float = 0, sy : Float = 0, sz : Float = 0, nx : Int = 20, ny : Int = 20, nz : Int = 20, dx : Float = 200, dy : Float = 200, dz : Float = 200)
    {
        super();
        
        this.nx = nx;this.ny = ny;this.nz = nz;
        this.dx = dx;this.dy = dy;this.dz = dz;
        this.sizeX = nx * dx;
        this.sizeY = ny * dy;
        this.sizeZ = nz * dz;
        this.minDelta = Math.min(dx, Math.min(dy, dz));
        
        startPoint = new Vector3D(sx, sy, sz);
        
        gridEntries = new Array<CollisionSystemGridEntry>();
        
        var len : Int = nx*ny*nz;
        for (j in 0...len){
            var gridEntry : CollisionSystemGridEntry = new CollisionSystemGridEntry(null);
            gridEntry.gridIndex = j;
            gridEntries.push(gridEntry);
        }
        
        overflowEntries = new CollisionSystemGridEntry(null);
        overflowEntries.gridIndex = -1;
    }
    
    private function calcIndex(i : Int, j : Int, k : Int) : Int
    {
        var _i : Int = i % nx;
        var _j : Int = j % ny;
        var _k : Int = k % nz;
        
        return (_i + nx * _j + (nx + ny) * _k);
    }
    
    private function calcGridForSkin3(colBody : RigidBody) : Vector3D
    {
        var i : Int;var j : Int;var k : Int;
        var sides : Vector3D = colBody.boundingBox.sideLengths;
        
        if ((sides.x > dx) || (sides.y > dy) || (sides.z > dz)) 
        {
            i = j = k = -1;
            return new Vector3D(i, j, k);
        }
        
        var min : Vector3D = colBody.boundingBox.minPos.clone();
        min.x = JMath3D.getLimiteNumber(min.x, startPoint.x, startPoint.x + sizeX);
        min.y = JMath3D.getLimiteNumber(min.y, startPoint.y, startPoint.y + sizeY);
        min.z = JMath3D.getLimiteNumber(min.z, startPoint.z, startPoint.z + sizeZ);
        
        i = Std.int(((min.x - startPoint.x) / dx) % nx);
        j = Std.int(((min.y - startPoint.y) / dy) % ny);
        k = Std.int(((min.z - startPoint.z) / dz) % nz);
        
        return new Vector3D(i, j, k);
    }
    
    
    public function calcGridForSkin6(colBody : RigidBody) : GridData
    {
        var tempStoreObject : GridData = new GridData();
        var i : Int;var j : Int;var k : Int;
        var fi : Float;var fj : Float;var fk : Float;
        
        var sides : Vector3D = colBody.boundingBox.sideLengths;
        
        if ((sides.x > dx) || (sides.y > dy) || (sides.z > dz)) 
        {
            //trace("calcGridForSkin6 -- Rigidbody to big for gridsystem - putting it into overflow list (lengths,type,id):", sides.x,sides.y,sides.z,colBody.type,colBody.id,colBody.boundingBox.minPos,colBody.boundingBox.maxPos);
            i = j = k = -1;
            fi = fj = fk = 0.0;
            tempStoreObject.i = i;tempStoreObject.j = j;tempStoreObject.k = k;tempStoreObject.fi = fi;tempStoreObject.fj = fj;tempStoreObject.fk = fk;
            return tempStoreObject;
        }
        
        var min : Vector3D = colBody.boundingBox.minPos.clone();
        
        min.x = JMath3D.getLimiteNumber(min.x, startPoint.x, startPoint.x + sizeX);
        min.y = JMath3D.getLimiteNumber(min.y, startPoint.y, startPoint.y + sizeY);
        min.z = JMath3D.getLimiteNumber(min.z, startPoint.z, startPoint.z + sizeZ);
        
        fi = (min.x - startPoint.x) / dx;
        fj = (min.y - startPoint.y) / dy;
        fk = (min.z - startPoint.z) / dz;
        
        i = Std.int(fi);
        j = Std.int(fj);
        k = Std.int(fk);
        
        if (i < 0) { i = 0; fi = 0.0; }
        else if (i >= nx) { i = 0; fi = 0.0; }
        else fi -= i;
        
        if (j < 0) { j = 0; fj = 0.0; }
        else if (j >= ny) { j = 0; fj = 0.0; }
        else fj -= j;
        
        if (k < 0) { k = 0; fk = 0.0; }
        else if (k >= nz) { k = 0; fk = 0.0; }
        else fk -= k;
        
        tempStoreObject.i = i;tempStoreObject.j = j;tempStoreObject.k = k;tempStoreObject.fi = fi;tempStoreObject.fj = fj;tempStoreObject.fk = fk;
        //trace(i,j,k,fi,fj,fk);
        //trace(colBody.x,colBody.y,colBody.z);
        return tempStoreObject;
    }
    
    
    private function calcGridIndexForBody(colBody : RigidBody) : Int
    {
        var tempStoreVector : Vector3D = calcGridForSkin3(colBody);
        
        if (tempStoreVector.x == -1) return -1;
        return calcIndex(Std.int(tempStoreVector.x), Std.int(tempStoreVector.y), Std.int(tempStoreVector.z));
    }
    
    override public function addCollisionBody(body : RigidBody) : Void
    {
        if (collBody.indexOf(body) < 0) 
            collBody.push(body);
        
        body.collisionSystem = this;
        
        // also do the grid stuff - for now put it on the overflow list
        var entry : CollisionSystemGridEntry = new CollisionSystemGridEntry(body);
        body.externalData = entry;
        
        // add entry to the start of the list
        CollisionSystemGridEntry.insertGridEntryAfter(entry, overflowEntries);
        collisionSkinMoved(body);
    }
    
    // not tested yet
    override public function removeCollisionBody(body : RigidBody) : Void
    {
        if (body.externalData != null) 
        {
            body.externalData.collisionBody = null;
            CollisionSystemGridEntry.removeGridEntry(body.externalData);
            body.externalData = null;
        }
        
        if (collBody.indexOf(body) >= 0) 
            collBody.splice(collBody.indexOf(body), 1);
    }
    
    override public function removeAllCollisionBodies() : Void
    {
        for (body in collBody){
            if (body.externalData != null) 
            {
                body.externalData.collisionBody = null;
                CollisionSystemGridEntry.removeGridEntry(body.externalData);
            }
        }

        //collBody.length = 0;
        collBody.splice(0, collBody.length);
    }
    
    
    // todo: only call when really moved, make it override public add into abstract ?
    override public function collisionSkinMoved(colBody : RigidBody) : Void
    {
        var entry : CollisionSystemGridEntry = colBody.externalData;
        if (entry == null) 
        {
            //trace("Warning rigidbody has grid entry null!");
            return;
        }
        
        var gridIndex : Int = calcGridIndexForBody(colBody);
        
        // see if it's moved grid
        if (gridIndex == entry.gridIndex) 
            return;
        
        //trace(gridIndex);
        var start : CollisionSystemGridEntry = null;
        //if (gridIndex >= 0**)
        if (gridEntries.length - 1 > gridIndex && gridIndex >= 0) // check if it's outside the gridspace, if so add to overflow  
            start = gridEntries[gridIndex]
        else 
            start = overflowEntries;
        
        CollisionSystemGridEntry.removeGridEntry(entry);
        CollisionSystemGridEntry.insertGridEntryAfter(entry, start);
    }
    
    
    private function getListsToCheck(colBody : RigidBody) : Array<CollisionSystemGridEntry>
    {
        var entries : Array<CollisionSystemGridEntry> = new Array<CollisionSystemGridEntry>();
        
        var entry : CollisionSystemGridEntry = colBody.externalData;
        if (entry == null) 
        {
            //trace("Warning skin has grid entry null!");
            return null;
        }
        
        // todo - work back from the mGridIndex rather than calculating it again...  
        var i : Int;
        var j : Int;
        var k : Int;
        var fi : Float;
        var fj : Float;
        var fk : Float;
        var tempStoreObject : GridData = calcGridForSkin6(colBody);
        i = tempStoreObject.i;j = tempStoreObject.j;k = tempStoreObject.k;fi = tempStoreObject.fi;fj = tempStoreObject.fj;fk = tempStoreObject.fk;
        
        if (i == -1) 
        {
            //trace("ADD ALL!");
            entries = gridEntries.concat([]);
            entries.push(overflowEntries);
            return entries;
        }
        
        // always add the overflow  
        entries.push(overflowEntries);
        
        var delta : Vector3D = colBody.boundingBox.sideLengths;  // skin.WorldBoundingBox.Max - skin.WorldBoundingBox.Min;  
        var maxI : Int = 1;
        var maxJ : Int = 1;
        var maxK : Int = 1;
        if (fi + (delta.x / dx) < 1) 
            maxI = 0;
        if (fj + (delta.y / dy) < 1) 
            maxJ = 0;
        if (fk + (delta.z / dz) < 1) 
            maxK = 0;
        
        // now add the contents of all grid boxes - their contents may extend beyond the bounds
        for (di in -1...maxI + 1){
            for (dj in -1...maxJ + 1){
                for (dk in -1...maxK + 1){
                    var thisIndex : Int = calcIndex(i + di, j + dj, k + dk);  // + ((nx*ny*nz)*0.5);  
                    //trace("ge", gridEntries.length);
                    if (gridEntries.length - 1 > thisIndex && thisIndex >= 0) {
                        var start : CollisionSystemGridEntry = gridEntries[thisIndex];
                        
                        //trace(thisIndex,gridEntries.length);
                        if (start != null && start.next != null) 
                        {
                            entries.push(start);
                        }
                    }
                }
            }
        }
        return entries;
    }
    
    
    override public function detectAllCollisions(bodies : Array<RigidBody>, collArr : Array<CollisionInfo>) : Void  // collisionFunctor:collCollisionFunctor , CollisionSkinPredicate2 collisionPredicate, float collTolerance)  
    {
        var info : CollDetectInfo;
        var fu : CollDetectFunctor;
        var bodyID : Int;
        var bodyType : String;
        _numCollisionsChecks = 0;
        
        for (body in bodies)
        {
            if (!body.isActive) 
                continue;
            
            bodyID = body.id;
            bodyType = body.type;
            
            var lists : Array<CollisionSystemGridEntry> = getListsToCheck(body);
            
            for (entry in lists)
            {
                
                entry = entry.next;
                while (entry != null){
                    if (body == entry.collisionBody) 
                        {entry = entry.next;continue;
                    };
                    
                    if (entry.collisionBody.isActive && bodyID > entry.collisionBody.id) 
                        {entry = entry.next;continue;
                    };
                    
                    if (checkCollidables(body, entry.collisionBody) && detectionFunctors[bodyType + "_" + entry.collisionBody.type] != null) 
                    {
                        info = new CollDetectInfo();
                        info.body0 = body;
                        info.body1 = entry.collisionBody;
                        fu = detectionFunctors[info.body0.type + "_" + info.body1.type];
                        fu.collDetect(info, collArr);
                        _numCollisionsChecks += 1;
                    }  //check collidables  
                    entry = entry.next;
                }  // loop over entries  
            }  // loop over lists  
        }  // loop over bodies  
    }
}
