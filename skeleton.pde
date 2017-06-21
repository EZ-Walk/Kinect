/*
Thomas Sanchez Lengeling.
 http://codigogenerativo.com/

 KinectPV2, Kinect for Windows v2 library for processing

 Skeleton depth tracking example
 */

import java.util.ArrayList;
import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;

float multiplyX = 3.5;
float multiplyY = 2;

Point lefty;

float leftyX;
float leftyY;

ArrayList<Grid> grid = new ArrayList();
ArrayList<Point> allPoints = new ArrayList();

//grid stuff
int gap = 30;
int dotSize = 1;
int roundedx;
int roundedy;
int gridSpring = 10; //lower number is strong


void setup() {
  size(1920, 1080, P3D);

  kinect = new KinectPV2(this);

  //Enables depth and Body tracking (mask image)
  kinect.enableDepthMaskImg(true);
  kinect.enableSkeletonDepthMap(true);

  kinect.init();
  
  gridSize(gap);
  
  for (int i = 0; i < roundedx+1; i++) {
    for (int j = 0; j < roundedy+1; j++) {
      grid.add(new Grid(gap*i, gap*j, dotSize, gridSpring));
      grid.add(new Grid(gap*i + (gap/4), gap * j, dotSize, gridSpring));
      grid.add(new Grid(gap*i, gap * j + (gap/4), dotSize, gridSpring));
      grid.add(new Grid(gap*i + (gap/2), gap * j, dotSize, gridSpring));
      grid.add(new Grid(gap*i, gap * j + (gap/2), dotSize, gridSpring));
      grid.add(new Grid(gap*i + (gap/4*3), gap * j, dotSize, gridSpring));
      grid.add(new Grid(gap*i, gap * j + (gap/4*3), dotSize, gridSpring));
    }
  }
  
  //drawing the hands, hard coded, for now.
  lefty = new Point(new PVector(width/4, height/4), 20, 5000);  
}

void draw() {
  background(0);

  //image(kinect.getDepthMaskImage(), 0, 0);

  //get the skeletons as an Arraylist of KSkeletons
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonDepthMap();

  //individual joints
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    //if the skeleton is being tracked compute the skleton joints
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      color col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);

      drawBody(joints);
      drawHandState(joints[KinectPV2.JointType_HandRight]);
      drawHandState(joints[KinectPV2.JointType_HandLeft]);
    }
  }
  
  for (Grid gridItem : grid) {
    gridItem.applyGravity();
    gridItem.keepShape();
    gridItem.display();
  }

  fill(255, 0, 0);
  text(frameRate, 50, 50);
}

//draw the body
void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);
  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);

  // Right Arm
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);

  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);

  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);

  //Single joints
  drawJoint(joints, KinectPV2.JointType_HandTipLeft);
  drawJoint(joints, KinectPV2.JointType_HandTipRight);
  drawJoint(joints, KinectPV2.JointType_FootLeft);
  drawJoint(joints, KinectPV2.JointType_FootRight);

  drawJoint(joints, KinectPV2.JointType_ThumbLeft);
  drawJoint(joints, KinectPV2.JointType_ThumbRight);

  drawJoint(joints, KinectPV2.JointType_Head);
}

//draw a single joint
void drawJoint(KJoint[] joints, int jointType) {
  pushMatrix();
  translate(joints[jointType].getX()*multiplyX, joints[jointType].getY()*multiplyY, joints[jointType].getZ());
  ellipse(0, 0, 25, 25);
  popMatrix();
}

//draw a bone from two joints
void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  pushMatrix();
  translate(joints[jointType1].getX()*multiplyX, joints[jointType1].getY()*multiplyY, joints[jointType1].getZ());
  ellipse(0, 0, 25, 25);
  popMatrix();
  line(joints[jointType1].getX()*multiplyX, joints[jointType1].getY()*multiplyY, joints[jointType1].getZ(), joints[jointType2].getX()*multiplyX, joints[jointType2].getY()*multiplyY, joints[jointType2].getZ());
}

//draw a ellipse depending on the hand state
void drawHandState(KJoint joint) {
  noStroke();
  handState(joint.getState());
  pushMatrix();
  translate(joint.getX()*multiplyX, joint.getY()*multiplyY, joint.getZ());
  ellipse(0, 0, 70, 70);
  lefty.newPos(joint.getX()*multiplyX, joint.getY() * multiplyY);
  popMatrix();
}

/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */

//Depending on the hand state change the color
void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    fill(0, 255, 0);
    lefty.changeMass(5000);
    break;
  case KinectPV2.HandState_Closed:
    fill(255, 0, 0);
    lefty.changeMass(50000);
    break;
  case KinectPV2.HandState_Lasso:
    fill(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    fill(100, 100, 100);
    break;
  }
}

void gridSize(int e) {
  float xCount = width/e;
  roundedx = Math.round(xCount);
  float yCount = height/e;
  roundedy = Math.round(yCount);
}