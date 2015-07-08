package jiglib.geometry;

import jiglib.geometry.JSegment;
import jiglib.geometry.JTriangle;

import jiglib.data.EdgeData;
import jiglib.data.OctreeCell;
import jiglib.data.TriangleVertexIndices;
import jiglib.math.JNumber3D;
import jiglib.math.JMath3D;
import jiglib.math.Vector3D;

class JOctree
{
    public var trianglesData(get, never) : Array<JIndexedTriangle>;
    public var verticesData(get, never) : Array<Vector3D>;

    
    // All our cells. The only thing guaranteed about this is that _cells[0] (if it exists) is the root cell.
    private var _cells : Array<OctreeCell>;
    private var _vertices : Array<Vector3D>;
    private var _triangles : Array<JIndexedTriangle>;
    private var _boundingBox : JAABox;
    
    private var _cellsToTest : Array<Int>;
    private var _testCounter : Int;
    
    public function new()
    {
        _testCounter = 0;
        _cells = new Array<OctreeCell>();
        _vertices = new Array<Vector3D>();
        _triangles = new Array<JIndexedTriangle>();
        _cellsToTest = new Array<Int>();
        _boundingBox = new JAABox();
    }
    
    private function get_trianglesData() : Array<JIndexedTriangle>{
        return _triangles;
    }
    
    public function getTriangle(iTriangle : Int) : JIndexedTriangle{
        return _triangles[iTriangle];
    }
    
    private function get_verticesData() : Array<Vector3D>{
        return _vertices;
    }
    public function getVertex(iVertex : Int) : Vector3D{
        return _vertices[iVertex];
    }
    
    public function boundingBox() : JAABox{
        return _boundingBox;
    }
    
    public function clear() : Void{
        //_cells.length = 0;
        //_vertices.length = 0;
        //_triangles.length = 0;

        _cells.splice(0, _cells.length);
        _vertices.splice(0, _vertices.length);
        _triangles.splice(0, _triangles.length);
    }
    
    // Add the triangles - doesn't actually build the octree
    public function addTriangles(vertices : Array<Vector3D>, numVertices : Int, triangleVertexIndices : Array<TriangleVertexIndices>, numTriangles : Int) : Void{
        clear();
        
        //_vertices = vertices.concat();
        _vertices = vertices.concat([]);
        
        var NLen : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        var i0 : Int;
        var i1 : Int;
        var i2 : Int;
        var dr1 : Vector3D;
        var dr2 : Vector3D;
        var N : Vector3D;
        var indexedTriangle : JIndexedTriangle;
        for (tri in triangleVertexIndices){
            i0 = tri.i0;
            i1 = tri.i1;
            i2 = tri.i2;
            
            dr1 = vertices[i1].subtract(vertices[i0]);
            dr2 = vertices[i2].subtract(vertices[i0]);
            N = dr1.crossProduct(dr2);
            NLen = N.length;
            
            if (NLen > tiny) 
            {
                indexedTriangle = new JIndexedTriangle();
                indexedTriangle.setVertexIndices(i0, i1, i2, _vertices);
                _triangles.push(indexedTriangle);
            }
        }
    }
    
    /* Builds the octree from scratch (not incrementally) - deleting
		 any previous tree.  Building the octree will involve placing
		 all triangles into the root cell.  Then this cell gets pushed
		 onto a stack of cells to examine. This stack will get parsed
		 and every cell containing more than maxTrianglesPerCell will
		 get split into 8 children, and all the original triangles in
		 that cell will get partitioned between the children. A
		 triangle can end up in multiple cells (possibly a lot!) if it
		 straddles a boundary. Therefore when intersection tests are
		 done tIndexedTriangle::m_counter can be set/tested using a
		 counter to avoid properly testing the triangle multiple times
		 (the counter _might_ wrap around, so when it wraps ALL the
		 triangle flags should be cleared! Could do this
		 incrementally...).*/
    public function buildOctree(maxTrianglesPerCell : Int, minCellSize : Float) : Void{
        _boundingBox.clear();
        
        for (vt in _vertices){
            _boundingBox.addPoint(vt);
        }
        
        //_cells.length = 0;
        _cells.splice(0, _cells.length);
        _cells.push(new OctreeCell(_boundingBox));
        
        var numTriangles : Int = _triangles.length;
        for (i in 0...numTriangles){
            _cells[0].triangleIndices.push(i);
        }
        
        var cellsToProcess : Array<Int> = new Array<Int>();
        cellsToProcess.push(0);
        
        var iTri : Int;
        var cellIndex : Int;
        var childCell : OctreeCell;
        while (cellsToProcess.length != 0){
            cellIndex = cellsToProcess.pop();
            
            if (_cells[cellIndex].triangleIndices.length <= maxTrianglesPerCell || _cells[cellIndex].AABox.getRadiusAboutCentre() < minCellSize) {
                continue;
            }
            for (i in 0...OctreeCell.NUM_CHILDREN){
                _cells[cellIndex].childCellIndices[i] = _cells.length;
                cellsToProcess.push(_cells.length);
                _cells.push(new OctreeCell(createAABox(_cells[cellIndex].AABox, i)));
                
                childCell = _cells[_cells.length - 1];
                numTriangles = _cells[cellIndex].triangleIndices.length;
                for (j in 0...numTriangles){
                    iTri = _cells[cellIndex].triangleIndices[j];
                    if (doesTriangleIntersectCell(_triangles[iTri], childCell)) 
                    {
                        childCell.triangleIndices.push(iTri);
                    }
                }
            }
            //_cells[cellIndex].triangleIndices.length = 0;
            _cells[cellIndex].triangleIndices.splice(0, _cells[cellIndex].triangleIndices.length);
        }
    }
    
    public function updateTriangles(vertices : Array<Vector3D>) : Void{
        //_vertices = vertices.concat();
        _vertices = vertices.concat([]);
        
        for (triangle in _triangles){
            triangle.updateVertexIndices(_vertices);
        }
    }
    
    /* Gets a list of all triangle indices that intersect an AABox. The vector passed in resized,
		 so if you keep it between calls after a while it won't grow any more, and this
		 won't allocate more memory.
		 Returns the number of triangles (same as triangles.size())*/
    public function getTrianglesIntersectingtAABox(triangles : Array<Int>, aabb : JAABox) : Int{
        if (_cells.length == 0) return 0;
        
        //_cellsToTest.length = 0;
        _cellsToTest.splice(0, _cellsToTest.length);
        _cellsToTest.push(0);
        
        incrementTestCounter();
        
        var cellIndex : Int;
        var nTris : Int;
        var cell : OctreeCell;
        var triangle : JIndexedTriangle;
        
        while (_cellsToTest.length != 0){
            cellIndex = _cellsToTest.pop();
            
            cell = _cells[cellIndex];
            
            if (!aabb.overlapTest(cell.AABox)) {
                continue;
            }
            
            if (cell.isLeaf()) {
                nTris = cell.triangleIndices.length;
                for (i in 0...nTris){
                    triangle = getTriangle(cell.triangleIndices[i]);
                    if (triangle.counter != _testCounter) {
                        triangle.counter = _testCounter;
                        if (aabb.overlapTest(triangle.boundingBox)) {
                            triangles.push(cell.triangleIndices[i]);
                        }
                    }
                }
            }
            else {
                for (i in 0...OctreeCell.NUM_CHILDREN){
                    _cellsToTest.push(cell.childCellIndices[i]);
                }
            }
        }
        return triangles.length;
    }
    
    public function dumpStats() : Void{
        var maxTris : Int = 0;
        var numTris : Int;
        var cellIndex : Int;
        var cell : OctreeCell;
        
        var cellsToProcess : Array<Int> = new Array<Int>();
        cellsToProcess.push(0);
        
        while (cellsToProcess.length != 0){
            cellIndex = cellsToProcess.pop();
            
            //cell = cell[cellIndex];
            cell = _cells[cellIndex];
            if (cell.isLeaf()) {
                
                numTris = cell.triangleIndices.length;
                if (numTris > maxTris) {
                    maxTris = numTris;
                }
            }
            else {
                for (i in 0...OctreeCell.NUM_CHILDREN){
                    //if ((cell.childCellIndices[i] >= 0) && (cell.childCellIndices[i] < as3hx.Compat.parseInt(_cells.length))) {
                    if ((cell.childCellIndices[i] >= 0) && (cell.childCellIndices[i] < Std.int(_cells.length))) {
                        cellsToProcess.push(cell.childCellIndices[i]);
                    }
                }
            }
        }
    }
    
    // Create a bounding box appropriate for a child, based on a parents AABox
    private function createAABox(aabb : JAABox, _id : Int) : JAABox{
        var dims : Vector3D = JNumber3D.getScaleVector(aabb.maxPos.subtract(aabb.minPos), 0.5);
        var offset : Vector3D;
        switch (_id)
        {
            case 0:
                offset = new Vector3D(1, 1, 1);
            case 1:
                offset = new Vector3D(1, 1, 0);
            case 2:
                offset = new Vector3D(1, 0, 1);
            case 3:
                offset = new Vector3D(1, 0, 0);
            case 4:
                offset = new Vector3D(0, 1, 1);
            case 5:
                offset = new Vector3D(0, 1, 0);
            case 6:
                offset = new Vector3D(0, 0, 1);
            case 7:
                offset = new Vector3D(0, 0, 0);
            default:
                offset = new Vector3D(0, 0, 0);
                //break;
        }
        
        var result : JAABox = new JAABox();
        result.minPos = aabb.minPos.add(new Vector3D(offset.x * dims.x, offset.y * dims.y, offset.z * dims.z));
        result.maxPos = result.minPos.add(dims);
        
        dims.scaleBy(0.00001);
        result.minPos = result.minPos.subtract(dims);
        result.maxPos = result.maxPos.add(dims);
        
        return result;
    }
    
    // Returns true if the triangle intersects or is contained by a cell
    private function doesTriangleIntersectCell(triangle : JIndexedTriangle, cell : OctreeCell) : Bool{
        if (!triangle.boundingBox.overlapTest(cell.AABox)) {
            return false;
        }
        if (cell.AABox.isPointInside(getVertex(triangle.getVertexIndex(0))) ||
            cell.AABox.isPointInside(getVertex(triangle.getVertexIndex(1))) ||
            cell.AABox.isPointInside(getVertex(triangle.getVertexIndex(2)))) {
            return true;
        }
        
        var tri : JTriangle = new JTriangle(getVertex(triangle.getVertexIndex(0)), getVertex(triangle.getVertexIndex(1)), getVertex(triangle.getVertexIndex(2)));
        var edge : EdgeData;
        var seg : JSegment;
        var edges : Array<EdgeData> = cell.egdes;
        var pts : Array<Vector3D> = cell.points;
        for (i in 0...12){
            edge = edges[i];
            seg = new JSegment(pts[edge.ind0], pts[edge.ind1].subtract(pts[edge.ind0]));
            if (tri.segmentTriangleIntersection(null, seg)) {
                return true;
            }
        }
        
        var pt0 : Vector3D;
        var pt1 : Vector3D;
        for (i in 0...3){
            pt0 = tri.getVertex(i);
            pt1 = tri.getVertex((i + 1) % 3);
            if (cell.AABox.segmentAABoxOverlap(new JSegment(pt0, pt1.subtract(pt0)))) {
                return true;
            }
        }
        return false;
    }
    
    /* Increment our test counter, wrapping around if necessary and zapping the triangle counters.
		 Const because we only modify mutable members.*/
    private function incrementTestCounter() : Void{
        ++_testCounter;
        if (_testCounter == 0) {
            var numTriangles : Int = _triangles.length;
            for (i in 0...numTriangles){
                _triangles[i].counter = 0;
            }
            _testCounter = 1;
        }
    }
}

