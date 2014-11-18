package jiglib.plugin;


import jiglib.math.Matrix3D;
import jiglib.math.Vector3D;

import jiglib.data.TriangleVertexIndices;

/**
	 * Represents a mesh from a 3D engine inside JigLib.
	 * Its implementation shold allow to get and set a Matrix3D on
	 * the original object.
	 *
	 * In the implementation, JMatrix3D should be translated into
	 * the type proper for a given engine.
	 */
interface ISkin3D
{
    
    
    /**
		 * Apply a matrix to the mesh.
		 */
    
    
    /**
		 * @return A matrix with the current transformation values of the mesh.
		 */
    var transform(get, set) : Matrix3D;    
    
    
    var vertices(get, never) : Array<Vector3D>;    
    var indices(get, never) : Array<TriangleVertexIndices>;

}

