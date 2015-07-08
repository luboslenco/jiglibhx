package jiglib.geometry;

import jiglib.data.CollOutData;
import jiglib.data.TriangleVertexIndices;
import jiglib.math.*;
import jiglib.physics.PhysicsState;
import jiglib.physics.RigidBody;
import jiglib.plugin.ISkin3D;

class JTriangleMesh extends RigidBody
{
    public var octree(get, never) : JOctree;

    private var _octree : JOctree;
    private var _maxTrianglesPerCell : Int;
    private var _minCellSize : Float;
    private var _skinVertices : Array<Vector3D>;
    
    public function new(skin : ISkin3D, initPosition : Vector3D, initOrientation : Matrix3D, maxTrianglesPerCell : Int = 10, minCellSize : Float = 10)
    {
        super(skin);
        
        currentState.position = initPosition.clone();
        currentState.orientation = initOrientation.clone();
        _maxTrianglesPerCell = maxTrianglesPerCell;
        _minCellSize = minCellSize;
        
        this.movable = false;
        
        if (skin != null) {
            _skinVertices = skin.vertices;
            createMesh(_skinVertices, skin.indices);
            
            _boundingBox = _octree.boundingBox().clone();
            skin.transform = JMatrix3D.getAppendMatrix3D(currentState.orientation, JMatrix3D.getTranslationMatrix(currentState.position.x, currentState.position.y, currentState.position.z));
        }
        
        _type = "TRIANGLEMESH";
    }
    
    /*Internally set up and preprocess all numTriangles. Each index
		 should, of course, be from 0 to numVertices-1. Vertices and
		 triangles are copied and stored internally.*/
    private function createMesh(vertices : Array<Vector3D>, triangleVertexIndices : Array<TriangleVertexIndices>) : Void{
        
        var len : Int = vertices.length;
        var vts : Array<Vector3D> = new Array<Vector3D>();
        
        var transform : Matrix3D = JMatrix3D.getTranslationMatrix(currentState.position.x, currentState.position.y, currentState.position.z);
        transform = JMatrix3D.getAppendMatrix3D(currentState.orientation, transform);
        
        for (_point in vertices){
            vts.push( transform.transformVector(_point) );
        }
        
        _octree = new JOctree();
        
        _octree.addTriangles(vts, vts.length, triangleVertexIndices, triangleVertexIndices.length);
        _octree.buildOctree(_maxTrianglesPerCell, _minCellSize);
    }
    
    private function get_octree() : JOctree{
        return _octree;
    }
    
    override public function segmentIntersect(out : CollOutData, seg : JSegment, state : PhysicsState) : Bool
    {
        var segBox : JAABox = new JAABox();
        segBox.addSegment(seg);
        
        var potentialTriangles : Array<Int> = new Array<Int>();
        var numTriangles : Int = _octree.getTrianglesIntersectingtAABox(potentialTriangles, segBox);
        
        var bestFrac : Float = JMath3D.NUM_HUGE;
        var tri : JTriangle;
        var meshTriangle : JIndexedTriangle;
        for (iTriangle in 0...numTriangles){
            meshTriangle = _octree.getTriangle(potentialTriangles[iTriangle]);
            
            tri = new JTriangle(_octree.getVertex(meshTriangle.getVertexIndex(0)), _octree.getVertex(meshTriangle.getVertexIndex(1)), _octree.getVertex(meshTriangle.getVertexIndex(2)));
            
            if (tri.segmentTriangleIntersection(out, seg)) {
                if (out.frac < bestFrac) {
                    bestFrac = out.frac;
                    out.position = seg.getPoint(bestFrac);
                    out.normal = meshTriangle.plane.normal;
                }
            }
        }
        out.frac = bestFrac;
        if (bestFrac < JMath3D.NUM_HUGE) {
            return true;
        }
        else {
            return false;
        }
    }
    
    override private function updateState() : Void
    {
        super.updateState();
        
        var len : Int = _skinVertices.length;
        var vts : Array<Vector3D> = new Array<Vector3D>();
        
        var transform : Matrix3D = JMatrix3D.getTranslationMatrix(currentState.position.x, currentState.position.y, currentState.position.z);
        transform = JMatrix3D.getAppendMatrix3D(currentState.orientation, transform);
        
        for (_point in _skinVertices){
            vts.push( transform.transformVector(_point) );
        }
        
        _octree.updateTriangles(vts);
        _octree.buildOctree(_maxTrianglesPerCell, _minCellSize);
        
        _boundingBox = _octree.boundingBox().clone();
    }
}
