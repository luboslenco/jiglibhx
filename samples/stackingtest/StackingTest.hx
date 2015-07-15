import h3d.scene.*;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.*;

typedef K = hxd.Key;

class StackingTest extends hxd.App {

	var physics:HeapsPhysics;
	var ground:RigidBody;
	var ballBody:Array<RigidBody>;
	var boxBody:Array<RigidBody>;

	var stats:Stats;

	override function init() {
		engine.backgroundColor = 0x0;

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
		mylight.y = 500;
		mylight.z = -700;
		mylight.params.set(0., 0., 0.000005);

		// set the ambient light to 30%
		s3d.lightSystem.ambientLight.set(0.3, 0.3, 0.3);

		s3d.camera.setFovX(60, 1);
		s3d.camera.pos.set(0, mylight.y, mylight.z);
		s3d.camera.up.set(0, 1, 0);
		s3d.camera.target.set(0, mylight.y-Math.sin(0.34906), mylight.z+Math.cos(0.34906));

		JConfig.solverType = "FAST";
		JConfig.doShockStep = true;
		physics = new HeapsPhysics(s3d, 8);

		// Objects
		ground = physics.createCube(cubeprim, greenMat, 500, 20, 500);
		ground.movable = false;
		ground.friction = 0.1;
		ground.restitution = 0.9;

		ballBody = new Array<RigidBody>();
		for (i in 0...15) {
			var ball = physics.createSphere(sphereprim, yellowMat, 22);
			ball.moveTo(new Vector3D(-100, 50*i+50, -200));
			ballBody.push(ball);
		}
		ballBody[0].mass = 10;
		physics.getMesh(ballBody[0]).material = redMat;

		boxBody = new Array<RigidBody>();
		var xNum:Int = 1;
		var yNum:Int = 15;
		var zNum:Int = 1;
		var boxSize:Vector3D = new Vector3D(50, 50, 50);
		var xstart:Float = -xNum * boxSize.x / 2;
		var ystart:Float = ground.currentState.position.y + boxSize.y / 2;
		var zstart:Float = -zNum * boxSize.z / 2;
		for (i in 0...xNum) {
			for (j in 0...yNum) {
				for (k in 0...zNum) {
					var box = physics.createCube(cubeprim, yellowMat, boxSize.x, boxSize.y, boxSize.z);
					box.mass = 0.1;
					box.moveTo(new Vector3D(xstart + (boxSize.x + 2) * i, ystart + boxSize.y * j, zstart + (boxSize.z + 2) * k));
					boxBody.push(box);
				}
			}
		}

		stats = new Stats(s2d, engine, physics);
	}

	private function resetBody():Void {
		var i:Int = 0;
		for (body in ballBody) {
			if (body.currentState.position.y < -200) {
				body.moveTo(new Vector3D(0, 1000 + (50 * i + 50), 0));
			}
			i++;
		}

		i = 0;
		for (body in boxBody) {
			if (body.currentState.position.y < -200) {
				body.moveTo(new Vector3D(0, 1000 + (50 * i + 50), 0));
			}
			i++;
		}
	}

	override function update( dt : Float ) {
		if (K.isDown(K.LEFT)) {
			ballBody[0].addWorldForce(new Vector3D(-150,0,0), ballBody[0].currentState.position);
		} else if (K.isDown(K.RIGHT)) {
			ballBody[0].addWorldForce(new Vector3D(150,0,0), ballBody[0].currentState.position);
		}

		if (K.isDown(K.UP)) {
			ballBody[0].addWorldForce(new Vector3D(0,0,150), ballBody[0].currentState.position);
		} else if (K.isDown(K.DOWN)) {
			ballBody[0].addWorldForce(new Vector3D(0,0,-150), ballBody[0].currentState.position);
		}

		if (K.isDown(K.SPACE)) {
			ballBody[0].addWorldForce(new Vector3D(0,150,0), ballBody[0].currentState.position);
		}

		physics.step(0.1);
		resetBody();

		stats.update(dt);
	}

	static function main() {

		// start the application
		new StackingTest();
	}

}
