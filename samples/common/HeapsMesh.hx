package;

import h3d.scene.Mesh;
import h3d.prim.Polygon;

import jiglib.math.Matrix3D;
import jiglib.math.Vector3D;

import jiglib.data.TriangleVertexIndices;
import jiglib.plugin.ISkin3D;

class HeapsMesh implements ISkin3D {

	public var transform(get, set) : Matrix3D;
	public var vertices(get, never) : Array<Vector3D>;
	public var indices(get, never) : Array<TriangleVertexIndices>;
	public var mesh(get, never) : Mesh;

	private var _mesh : Mesh;
	private var _translationOffset : Vector3D;
	private var _scale : Vector3D;
	private var _transform : Matrix3D = new Matrix3D();

	public function new(do3d : Mesh, offset : Vector3D = null)
	{
		this._mesh = do3d;

		_transform.identity();

		if (offset != null) {
			_translationOffset = offset.clone();
		}
		if (do3d.scaleX != 1 || do3d.scaleY != 1 || do3d.scaleZ != 1) {
			_scale = new Vector3D(do3d.scaleX, do3d.scaleY, do3d.scaleZ);
		}
	}

	function get_transform():Matrix3D {

		return _transform;
	}

	function set_transform(m:Matrix3D) {
		_transform.identity();
		if (_translationOffset != null) _transform.appendTranslation(_translationOffset.x, _translationOffset.y, _translationOffset.z);
		if (_scale != null) _transform.appendScale(_scale.x, _scale.y, _scale.z);
		_transform.append(m);

		var decom = _transform.decompose();
		var pos = decom[0], rot = decom[1], scale = decom[2];
		_mesh.setPos(pos.x, pos.y, pos.z);
		_mesh.setRotate(rot.x, rot.y, rot.z);
		_mesh.scaleX = scale.x;
		_mesh.scaleY = scale.y;
		_mesh.scaleZ = scale.z;

		return _transform;
	}

	function get_mesh():Mesh {
		return _mesh;
	}

	function get_vertices():Array<Vector3D> {
		var result:Array<Vector3D> = new Array<Vector3D>();

		var vts = cast(_mesh.primitive,Polygon).points;
		for (vt in vts) {
			result.push(new Vector3D(vt.x, vt.y, vt.z));
		}
		return result;
	}

	function get_indices():Array<TriangleVertexIndices>{
		var result:Array<TriangleVertexIndices>=new Array<TriangleVertexIndices>();

		//TODO:
		//var ids:Array<UInt>=_mesh.geometry.subGeometries[0].indexData;
		//for(var i:uint=0;i<ids.length;i+=3){
		//	result.push(new TriangleVertexIndices(ids[i],ids[i+1],ids[i+2]));
		//}
		return result;
	}
}
