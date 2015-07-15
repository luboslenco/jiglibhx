package;

import h3d.mat.MeshMaterial;
import h3d.prim.*;
import h3d.scene.*;

import hxd.BitmapData;
import jiglib.math.Matrix3D;
import jiglib.math.Vector3D;

import jiglib.geometry.JBox;
import jiglib.geometry.JPlane;
import jiglib.geometry.JSphere;
import jiglib.geometry.JTerrain;
import jiglib.geometry.JTriangleMesh;
import jiglib.physics.RigidBody;
import jiglib.plugin.AbstractPhysics;

class HeapsPhysics extends AbstractPhysics {

	private var parent:Scene;

	public function new(parent:Scene, speed:Float = 1) {
		this.parent = parent;
		super(speed);
	}

	public function getMesh(body:RigidBody):Mesh {
		if (body.skin != null) {
			return cast(body.skin, HeapsMesh).mesh;
		} else {
			return null;
		}
	}

	public function createGround(quadprim:Quads, material:MeshMaterial, width:Int=500, height:Int=500, level:Float = 0):RigidBody {
		var ground:Mesh = new Mesh(quadprim, material, parent);
		ground.scaleX = width;
		ground.scaleZ = height;

		var jGround:JPlane = new JPlane(new HeapsMesh(ground), new Vector3D(0, 1, 0));
		jGround.z = level;
		jGround.movable = false;
		addBody(jGround);
		return jGround;
	}

	public function createCube(cubeprim:Cube, material:MeshMaterial, width:Float=500, height:Float=500, depth:Float=500):RigidBody {
		var cube:Mesh = new Mesh(cubeprim, material, parent);
		cube.scaleX = width;
		cube.scaleY = height;
		cube.scaleZ = depth;

		var jBox:JBox = new JBox(new HeapsMesh(cube), width, depth, height);
		addBody(jBox);
		return jBox;
	}

	public function createSphere(sphereprim:Sphere, material:MeshMaterial, radius:Float = 50):RigidBody {
		var sphere:Mesh = new Mesh(sphereprim, material, parent);
		sphere.scale(radius);

		var jsphere:JSphere = new JSphere(new HeapsMesh(sphere), radius);
		addBody(jsphere);
		return jsphere;
	}

	public function createTerrain(material : MeshMaterial, heightMap : BitmapData, width : Float = 1000, height : Float = 100, depth : Float = 1000, segmentsW : UInt = 30, segmentsH : UInt = 30, maxElevation:UInt = 255, minElevation:UInt = 0):JTerrain {
		var terrainMap:HeapsTerrain = new HeapsTerrain(heightMap, width, height, depth, segmentsW, segmentsH, maxElevation, minElevation);
		terrainMap.unindex();
		terrainMap.addNormals();
		var terrainMesh:Mesh = new Mesh(terrainMap, material, parent);

		var terrain:JTerrain = new JTerrain(terrainMap);
		addBody(terrain);

		return terrain;
	}

	/*
	public function createMesh(skin:Mesh,initPosition:Vector3D,initOrientation:Matrix3D,maxTrianglesPerCell:int = 10, minCellSize:Float = 10):JTriangleMesh{
		var mesh:JTriangleMesh=new JTriangleMesh(new Away3D4Mesh(skin),initPosition,initOrientation,maxTrianglesPerCell,minCellSize);
		addBody(mesh);

		return mesh;
	}
	*/
}
