import java.io.File;

PShape base, shoulder, upArm, loArm, endR;
float rotX, rotY=-PI/2;
float posX=40, posY=40, posZ=0;
float alpha , beta, gamma;
float F = 50;
float T = 70;
float millisOld, gTime, gSpeed = 15;
float time = 10;

ArrayList<PVector> coords = new ArrayList<PVector>();

void IK(){
	float X = 40;
	float Y = 40;
	float Z = 0;

	float L = sqrt(Y*Y+X*X);
	float dia = sqrt(Z*Z+L*L);
	alpha = PI/2-(atan2(L, Z)+acos((T*T-F*F-dia*dia)/(-2*F*dia)));
	beta = -PI+acos((dia*dia-T*T-F*F)/(-2*F*T));
	gamma = atan2(Y, X);
}

void setTime(){
	gTime += ((float)millis()/1000 - millisOld)*(gSpeed/time);
	if(gTime >= time) gTime = 0;
	millisOld = (float)millis()/1000;
}

void writePos(){
	IK();
	setTime();
}

int n = 2;
float[]  Xsphere = new float[n];
float[]  Ysphere = new float[n];
float[]  Zsphere = new float[n];

void setup(){
	size(1200, 800, OPENGL);

	base = loadShape("r5.obj");
	shoulder = loadShape("r1.obj");
	upArm = loadShape("r2.obj");
	loArm = loadShape("r3.obj");
	endR = loadShape("r4.obj");

	shoulder.disableStyle();
	upArm.disableStyle();
	loArm.disableStyle();
}

void draw(){
	writePos();
	background(150);
	smooth();
	lights();
	directionalLight(51, 102, 126, -1, 0, 0);

	for (int i=0; i<Xsphere.length -1; i++){
		Xsphere[i] = Xsphere[i + 1];
		Ysphere[i] = Ysphere[i + 1];
		Zsphere[i] = Zsphere[i + 1];
	}
	
  Xsphere[Xsphere.length - 1] = posX;
	Ysphere[Ysphere.length - 1] = posY;
	Zsphere[Zsphere.length - 1] = posZ;
	
	noStroke();

	translate(width/2, height/2);
	rotateX(rotX);
	rotateY(-rotY);
	scale(-3);

	for (int i=0; i < Xsphere.length; i++) {
		pushMatrix();
		translate(-Ysphere[i], -Zsphere[i]-11, -Xsphere[i]);
		fill(#910100, 100);
		sphere(0.5);
		popMatrix();
	}

	drawCoordinates();

	fill(#FFE308);
	translate(0,-40,0);
	shape(base);

	translate(0 , 4, 0);
	rotateY(gamma);
	shape(shoulder);

	translate(0, 25, 0);
	rotateY(PI);
	rotateX(alpha);
	shape(upArm);

	translate(0, 0, 50);
	rotateY(PI);
	rotateX(beta);
	shape(loArm);

	translate(0, 0, -50);
	rotateY(PI);
	shape(endR);
	
}

void mouseDragged(){
	rotY -= (mouseX - pmouseX) * 0.01;
	rotX -= (mouseY - pmouseY) * 0.01;

}

void drawCoordinates(){
	pushMatrix();
	translate(0,0,0);
	rotateX(PI);
	rotateY(PI/2 * 3);
	String info = "X: " + nf(posX,0,2) +" "+ "Y: " + nf(posY,0,2) +" "+ "Z: " + nf(posZ,0,2);
	text(info,70,-50,-100);
	popMatrix();
}
