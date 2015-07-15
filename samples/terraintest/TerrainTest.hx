import h3d.scene.*;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.*;

typedef K = hxd.Key;

class TerrainTest extends hxd.App {

	var physics:HeapsPhysics;
	var terrain:JTerrain;
	var ballBody:Array<RigidBody>;
	var boxBody:Array<RigidBody>;

	var stats:Stats;

	override function init() {
		engine.backgroundColor = 0x222266;

		// Primitives
		// creates a new unit cube
		var cubeprim = new h3d.prim.Cube();
		// translate it so its center will be at the center of the cube
		cubeprim.translate( -0.5, -0.5, -0.5);
		// unindex the faces to create hard edges normals
		cubeprim.unindex();
		// add face normals
		cubeprim.addNormals();
		// add texture coordinates
		cubeprim.addUVs();

		var sphereprim = new h3d.prim.Sphere(16, 12);
		sphereprim.addNormals();
		sphereprim.addUVs();


		// Materials
		var greenMat = new h3d.mat.MeshMaterial();
		greenMat.color.setColor(0x77ee77);
		greenMat.mainPass.enableLights = true;

		var yellowMat = new h3d.mat.MeshMaterial();
		yellowMat.color.setColor(0xeeee00);
		yellowMat.mainPass.enableLights = true;

		var redMat = new h3d.mat.MeshMaterial();
		redMat.color.setColor(0xff8888);
		redMat.mainPass.enableLights = true;


		// adds a point light to the scene
		var mylight = new h3d.scene.PointLight(s3d);
		mylight.color.set(1, 1, 1);
		mylight.y = 800;
		mylight.z = -800;
		mylight.params.set(0., 0., 0.000005);

		// set the ambient light to 30%
		s3d.lightSystem.ambientLight.set(0.3, 0.3, 0.3);

		s3d.camera.setFovX(60, 1);
		s3d.camera.pos.set(0, mylight.y, mylight.z);
		s3d.camera.up.set(0, 1, 0);
		s3d.camera.target.set(0, mylight.y-Math.sin(0.6981317), mylight.z+Math.cos(0.6981317));

		JConfig.solverType = "FAST";
		physics = new HeapsPhysics(s3d, 8);
		physics.engine.setCollisionSystem(true, -500, -500, -500, 20, 20, 20, 100, 100, 100);

		// Objects
		terrain = physics.createTerrain(greenMat, hxd.Res.heightmap3.toBitmap(), 1000, 300, 1000, 50, 50, 300, 0);

		ballBody = new Array<RigidBody>();
		for (i in 0...20) {
			var ball = physics.createSphere(sphereprim, yellowMat, 20);
			ball.moveTo(new Vector3D(-300+600*Math.random(), 500+500*Math.random(), -300+600*Math.random()));
			ballBody.push(ball);
		}
		ballBody[0].mass = 10;
		physics.getMesh(ballBody[0]).material = redMat;

		boxBody = new Array<RigidBody>();
		for (i in 0...20) {
			var box = physics.createCube(cubeprim, yellowMat, 50, 30, 40);
			box.moveTo(new Vector3D(-300+600*Math.random(), 500+500*Math.random(), -300+600*Math.random()));
			boxBody.push(box);
		}

		stats = new Stats(s2d, engine, physics);
	}

	override function update( dt : Float ) {
		if (K.isDown(K.LEFT)) {
			ballBody[0].addWorldForce(new Vector3D(-100,0,0), ballBody[0].currentState.position);
		} else if (K.isDown(K.RIGHT)) {
			ballBody[0].addWorldForce(new Vector3D(100,0,0), ballBody[0].currentState.position);
		}

		if (K.isDown(K.UP)) {
			ballBody[0].addWorldForce(new Vector3D(0,0,100), ballBody[0].currentState.position);
		} else if (K.isDown(K.DOWN)) {
			ballBody[0].addWorldForce(new Vector3D(0,0,-100), ballBody[0].currentState.position);
		}

		if (K.isDown(K.SPACE)) {
			ballBody[0].addWorldForce(new Vector3D(0,100,0), ballBody[0].currentState.position);
		}

		physics.step(0.2);

		stats.update(dt);
	}

	static function main() {

		// Initialize embedded resources
		hxd.Res.initEmbed();

		// start the application
		new TerrainTest();
	}

}
