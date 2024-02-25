PShape base, shoulder, upArm, loArm, endR;
float rotX, rotY=-PI/2;
float posX=-88, posY=-40, posZ=-7;
float alpha , beta, gamma;
float F = 50;
float T = 70;
float millisOld, gTime, gSpeed = 15;
float time = 10;

ArrayList<PVector> coords = new ArrayList<PVector>();
ArrayList<Integer> isfilled = new ArrayList<Integer>();

void loadData(){
	String filename = "new_coordinates.json";
	JSONArray file_coords = loadJSONArray(filename);
	int resize = 1;

	for(int i=0; i<file_coords.size(); i++) {
		JSONArray innerArray = file_coords.getJSONArray(i);
		float x = innerArray.getFloat(0)/resize;
		float y = innerArray.getFloat(1)/resize;
		float z = innerArray.getFloat(2)/resize;

		int f = innerArray.getInt(3);
		coords.add(new PVector(x,y,z));
		isfilled.add(f);
	}

}

void IK(){
	float X = posX;
	float Y = posY;
	float Z = posZ;

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

int i = 0;
int filled = 1;
void writePos(){
	IK();
	setTime();
	if(i<coords.size()){
		posX = coords.get(i).x; 
		posY = coords.get(i).y;
		posZ = coords.get(i).z; 
		filled = isfilled.get(i);
		i=i+3;		
	}else{
		i=0;
	}
}

int n = 700;
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

	loadData();
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
	
	if(filled == 1){
		Xsphere[Xsphere.length - 1] = posX;
		Ysphere[Ysphere.length - 1] = posY;
		Zsphere[Zsphere.length - 1] = posZ;

	}
	
	noStroke();

	translate(width/2, height/2);
	rotateX(rotX);
	rotateY(-rotY);
	scale(-2);

	for (int i=0; i < Xsphere.length; i++) {
		pushMatrix();
		translate(-Ysphere[i], -Zsphere[i]-11, -Xsphere[i]);
		fill(#920100, 100);
		sphere(2);
		popMatrix();
	}

	drawAxis();

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

void drawAxis(){
	pushMatrix();
	translate(0,0,0);
	rotateX(PI);
	rotateY(PI/2 * 3);
	String info = "X: " + nf(posX,0,2) +" "+ "Y: " + nf(posY,0,2) +" "+ "Z: " + nf(posZ,0,2);
	text(info,-100,-100,-100); 
	//stroke(#01016f); //blue x-axis
	//line(0,0,-500,0,0,0);
	//stroke(#d8031c); //red y-axis
	//line(0,0,0,-500,0,0);
	popMatrix();
}
