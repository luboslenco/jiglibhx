import h3d.col.Point;
import h3d.prim.UV;
import h3d.scene.*;

import jiglib.cof.JConfig;
import jiglib.geometry.*;
import jiglib.math.*;
import jiglib.physics.*;
import jiglib.vehicles.JCar;

typedef K = hxd.Key;

class CarDrive extends hxd.App {

	var physics:HeapsPhysics;
	var boxBody:Array<RigidBody>;
	var carBody:JCar;

	var carMesh:Mesh;
	var steerFR:Object;
	var steerFL:Object;
	var wheelFR:Mesh;
	var wheelFL:Mesh;
	var wheelBR:Mesh;
	var wheelBL:Mesh;

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

		var carMat = new h3d.mat.MeshMaterial();
		carMat.texture = hxd.Res.fskin.toTexture();
		carMat.mainPass.enableLights = true;


		// adds a point light to the scene
		var mylight = new h3d.scene.PointLight(s3d);
		mylight.color.set(1, 1, 1);
		mylight.y = 1000;
		mylight.z = -1000;
		mylight.params.set(0., 0., 0.000001);

		// set the ambient light to 30%
		s3d.lightSystem.ambientLight.set(0.3, 0.3, 0.3);

		s3d.camera.setFovX(60, 1);
		s3d.camera.up.set(0, 1, 0);
		s3d.camera.pos.set(-300, 300, 300);
		s3d.camera.target.set(0, 0, 0);

		physics = new HeapsPhysics(s3d, 6);

		// Objects
		var ground = physics.createGround(quadprim, greenMat, 1000, 1000);

		boxBody = new Array<RigidBody>();
		for (i in 0...3) {
			var box = physics.createCube(cubeprim, yellowMat, 60, 50, 80);
			box.moveTo(new Vector3D(0, 10 + (50 * i + 50), 0));
			boxBody.push(box);
		}

		// Load the car model
		var scale:Float = 40;
		var container = HeapsOBJModel.load(hxd.Res.car_obj.entry.getBytes().toString());
		carMesh = new Mesh(container.objects[1], carMat, s3d);
		carMesh.scale(scale);
		steerFL = new Object(carMesh);
		steerFR = new Object(carMesh);
		wheelFL = new Mesh(container.objects[0], carMat, steerFL);
		wheelFR = new Mesh(container.objects[3], carMat, steerFR);
		wheelBL = new Mesh(container.objects[4], carMat, carMesh);
		wheelBR = new Mesh(container.objects[2], carMat, carMesh);

		wheelBL.setPos(-48/scale, -20/scale, -84/scale);
		wheelBR.setPos(48/scale, -20/scale, -84/scale);
		steerFL.setPos(-48/scale, -20/scale, 68/scale);
		steerFR.setPos(48/scale, -20/scale, 68/scale);

		// Set up the car physics
		carBody = new JCar(null);
		carBody.setCar(40, 1, 500);
		carBody.chassis.moveTo(new Vector3D(-200, 200, 0));
		carBody.chassis.rotationY = 90;
		carBody.chassis.mass = 10;
		carBody.chassis.sideLengths = new Vector3D(105, 40, 220);
		physics.addBody(carBody.chassis);

		carBody.setupWheel("WheelFL", new Vector3D(-48, -20, 68), 1.3, 1.3, 6, 20, 0.5, 0.5, 2);
		carBody.setupWheel("WheelFR", new Vector3D(48, -20, 68), 1.3, 1.3, 6, 20, 0.5, 0.5, 2);
		carBody.setupWheel("WheelBL", new Vector3D(-48, -20, -84), 1.3, 1.3, 6, 20, 0.5, 0.5, 2);
		carBody.setupWheel("WheelBR", new Vector3D(48, -20, -84), 1.3, 1.3, 6, 20, 0.5, 0.5, 2);

		stats = new Stats(s2d, engine, physics);
	}

	private function updateCarSkin():Void
	{
		if (carBody == null)
			return;

		var decom = JMatrix3D.getAppendMatrix3D(carBody.chassis.currentState.orientation, JMatrix3D.getTranslationMatrix(carBody.chassis.currentState.position.x, carBody.chassis.currentState.position.y, carBody.chassis.currentState.position.z)).decompose();
		var pos = decom[0], rot = decom[1], scale = decom[2];
		carMesh.setPos(pos.x, pos.y, pos.z);
		carMesh.setRotate(rot.x, rot.y, rot.z);

		var degToRad = Math.PI / 180;

		wheelFL.rotate(carBody.wheels["WheelFL"].getRollAngle() * degToRad, 0, 0);
		wheelFR.rotate(carBody.wheels["WheelFR"].getRollAngle() * degToRad, 0, 0);
		wheelBL.rotate(carBody.wheels["WheelBL"].getRollAngle() * degToRad, 0, 0);
		wheelBR.rotate(carBody.wheels["WheelBR"].getRollAngle() * degToRad, 0, 0);

		steerFL.setRotate(0, carBody.wheels["WheelFL"].getSteerAngle() * degToRad, 0);
		steerFR.setRotate(0, carBody.wheels["WheelFR"].getSteerAngle() * degToRad, 0);

		steerFL.y = carBody.wheels["WheelFL"].getActualPos().y / 40;
		steerFR.y = carBody.wheels["WheelFR"].getActualPos().y / 40;
		wheelBL.y = carBody.wheels["WheelBL"].getActualPos().y / 40;
		wheelBR.y = carBody.wheels["WheelBR"].getActualPos().y / 40;

		s3d.camera.pos.set(carMesh.x, carMesh.y+500, carMesh.z-500);
		s3d.camera.target.set(carMesh.x, carMesh.y, carMesh.z);
	}

	override function update( dt : Float ) {
		if (K.isDown(K.UP)) {
			carBody.setAccelerate(1);
		} else if (K.isDown(K.DOWN)) {
			carBody.setAccelerate(-1);
		} else {
			carBody.setAccelerate(0);
		}

		if (K.isDown(K.LEFT)) {
			carBody.setSteer(["WheelFL", "WheelFR"], -1);
		} else if (K.isDown(K.RIGHT)) {
			carBody.setSteer(["WheelFL", "WheelFR"], 1);
		} else {
			carBody.setSteer(["WheelFL", "WheelFR"], 0);
		}

		if (K.isDown(K.SPACE)) {
			carBody.setHBrake(1);
		} else {
			carBody.setHBrake(0);
		}

		updateCarSkin();
		physics.step(0.1);

		stats.update(dt);
	}

	static function main() {

		hxd.Res.initEmbed();

		// start the application
		new CarDrive();
	}

}
