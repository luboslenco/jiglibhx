import h3d.col.Point;
import h3d.prim.UV;
import h3d.scene.*;

import jiglib.cof.JConfig;
import jiglib.events.JCollisionEvent;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.*;

typedef K = hxd.Key;

class CollisionEventTest extends hxd.App {

	var physics:HeapsPhysics;
	var ground:RigidBody;
	var ball:RigidBody;
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

		var quadprim = new h3d.prim.Quads(
							[new Point(-0.5,0,-0.5), new Point(-0.5,0,0.5), new Point(0.5,0,-0.5), new Point(0.5,0,0.5)],
							[new UV(0,0), new UV(0,1), new UV(1,0), new UV(1,1)],
							[new Point(0,1,0), new Point(0,1,0), new Point(0,1,0), new Point(0,1,0)]);


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

		var blueMat = new h3d.mat.MeshMaterial();
		blueMat.color.setColor(0x0000ff);
		blueMat.mainPass.enableLights = true;


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
		s3d.camera.up.set(0, 0, 1);
		s3d.camera.target.set(0, mylight.y-Math.sin(0.34906), mylight.z+Math.cos(0.34906));

		JConfig.solverType = "FAST";
		JConfig.doShockStep = true;
		physics = new HeapsPhysics(s3d, 8);

		// Objects
		ground = physics.createGround(quadprim, greenMat);

		function collisionStart(event:JCollisionEvent):Void {
			if (event.body != ground) {
				physics.getMesh(event.body).material = blueMat;
			}
		}
		function collisionEnd(event:JCollisionEvent):Void {
			if (event.body != ground) {
				physics.getMesh(event.body).material = yellowMat;
			}
		}
		ball = physics.createSphere(sphereprim, redMat, 22);
		ball.moveTo(new Vector3D(-100, 50, -200));
		#if JIGLIB_FLASH_EVENTS
		ball.addEventListener(JCollisionEvent.COLLISION_START, collisionStart);
		ball.addEventListener(JCollisionEvent.COLLISION_END, collisionEnd);
		#else
		ball.onCollisionStart = collisionStart;
		ball.onCollisionEnd = collisionEnd;
		#end

		boxBody = new Array<RigidBody>();
		for (i in 0...4) {
			var box = physics.createCube(cubeprim, yellowMat, 50, 50, 50);
			box.moveTo(new Vector3D(-150+100*i, 50, 0));
			boxBody.push(box);
		}

		stats = new Stats(s2d, engine, physics);
	}

	override function update( dt : Float ) {
		if (K.isDown(K.LEFT)) {
			ball.addWorldForce(new Vector3D(-10,0,0), ball.currentState.position);
		}
		if (K.isDown(K.RIGHT)) {
			ball.addWorldForce(new Vector3D(10,0,0), ball.currentState.position);
		}
		if (K.isDown(K.UP)) {
			ball.addWorldForce(new Vector3D(0,0,10), ball.currentState.position);
		}
		if (K.isDown(K.DOWN)) {
			ball.addWorldForce(new Vector3D(0,0,-10), ball.currentState.position);
		}
		if (K.isDown(K.SPACE)) {
			ball.addWorldForce(new Vector3D(0,10,0), ball.currentState.position);
		}

		physics.step(0.1);

		stats.update(dt);
	}

	static function main() {

		// start the application
		new CollisionEventTest();
	}

}
