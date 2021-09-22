/******************************************************************************\
* Copyright (C) 2012-2018 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <iostream>
//#include <Windows.h>
#include <cstring>
#include<chrono>
#include<thread>
//#include "LeapC++.h"
#include "../LeapSDK/include/LeapC.h"
#include <Springhead.h>
#include <Framework/SprFWApp.h>
//#include "FWFileLoaderSample.h"
#include "../src/Samples/SampleApp.h"
extern "C" {
  #include "ExampleConnection.h"
}
  //#include "Handler.h"

//using namespace Leap;
using namespace Spr;
using namespace std;
PHSolidIf* soFloor;

static LEAP_CONNECTION* connectionHandle;


vector<PHSolidIf*> soBox;

double					floorShakeAmplitude;
//Vec3d spr_palm;

//Vec3d spr_hito0;
//Quaterniond hito_orientation;


static void OnConnect() {
  printf("Connected.\n");
}
/** Callback for when a device is found. */
static void OnDevice(const LEAP_DEVICE_INFO* props) {
  printf("Found device %s.\n", props->serial);
}
//
///** Callback for when a frame of tracking data is available. */
//static void OnFrame(const LEAP_TRACKING_EVENT* frame) {
//  //printf("Frame %lli with %i hands.\n", (long long int)frame->info.frame_id, frame->nHands);
//
//  for (uint32_t h = 0; h < frame->nHands; h++) {
//    LEAP_HAND* hand = &frame->pHands[h];
//
//    oya_position = Vec3d((double)hand->thumb.bones->prev_joint.x / 10, (double)hand->thumb.bones->prev_joint.y / 10, (double)hand->thumb.bones->prev_joint.z / 10);
//    oya_orientation = Quaterniond((double)hand->thumb.bones->rotation.w, (double)hand->thumb.bones->rotation.x, (double)hand->thumb.bones->rotation.y, (double)hand->thumb.bones->rotation.z);
//    cout << oya_orientation << endl;
//    //更新
//    //soOya0->SetFramePosition(oya_position);
//    //soOya0->SetOrientation(oya_orientation);
//
//
//    //oya_position.x = hand->thumb.bones->prev_joint.x/10;
//    //oya_position.y = hand->thumb.bones->prev_joint.y/10;
//    //oya_position.z = hand->thumb.bones->prev_joint.z/10;
//    //oya_orientation.x = hand->thumb.bones->rotation.x;
//    //oya_orientation.y = hand->thumb.bones->rotation.y;
//    //oya_orientation.z = hand->thumb.bones->rotation.z;
//    //oya_orientation.w = hand->thumb.bones->rotation.w;
//    //soOya0->SetOrientation(oya_orientation);
//    //soOya0->SetFramePosition(oya_position);
//  }
//}

//static void OnImage(const LEAP_IMAGE_EVENT* image) {
//  printf("Image %lli  => Left: %d x %d (bpp=%d), Right: %d x %d (bpp=%d)\n",
//    (long long int)image->info.frame_id,
//    image->image[0].properties.width, image->image[0].properties.height, image->image[0].properties.bpp * 8,
//    image->image[1].properties.width, image->image[1].properties.height, image->image[1].properties.bpp * 8);
//}

static void OnLogMessage(const eLeapLogSeverity severity, const int64_t timestamp,
  const char* message) {
  const char* severity_str;
  switch (severity) {
  case eLeapLogSeverity_Critical:
    severity_str = "Critical";
    break;
  case eLeapLogSeverity_Warning:
    severity_str = "Warning";
    break;
  case eLeapLogSeverity_Information:
    severity_str = "Info";
    break;
  default:
    severity_str = "";
    break;
  }
  printf("[%s][%lli] %s\n", severity_str, (long long int)timestamp, message);
}

static void* allocate(uint32_t size, eLeapAllocatorType typeHint, void* state) {
  void* ptr = malloc(size);
  return ptr;
}

static void deallocate(void* ptr, void* state) {
  if (!ptr)
    return;
  free(ptr);
}

void OnPointMappingChange(const LEAP_POINT_MAPPING_CHANGE_EVENT* change) {
  if (!connectionHandle)
    return;

  uint64_t size = 0;
  if (LeapGetPointMappingSize(*connectionHandle, &size) != eLeapRS_Success || !size)
    return;

  LEAP_POINT_MAPPING* pointMapping = (LEAP_POINT_MAPPING*)malloc(size);
  if (!pointMapping)
    return;

  if (LeapGetPointMapping(*connectionHandle, pointMapping, &size) == eLeapRS_Success &&
    pointMapping->nPoints > 0) {
    printf("Managing %u points as of frame %lld at %lld\n", pointMapping->nPoints, (long long int)pointMapping->frame_id, (long long int)pointMapping->timestamp);
  }
  free(pointMapping);
}
//
//void OnHeadPose(const LEAP_HEAD_POSE_EVENT* event) {
//  printf("Head pose:\n");
//  printf("    Head position (%f, %f, %f).\n",
//    event->head_position.x,
//    event->head_position.y,
//    event->head_position.z);
//  printf("    Head orientation (%f, %f, %f, %f).\n",
//    event->head_orientation.w,
//    event->head_orientation.x,
//    event->head_orientation.y,
//    event->head_orientation.z);
//}

class MyApp : public SampleApp {
public:
  /// ページID
  enum {
    MENU_MAIN = MENU_SCENE,
  };
  /// アクションID
  enum {
    ID_BOX,
    ID_CAPSULE,
    ID_ROUNDCONE,
    ID_SPHERE,
    ID_ELLIPSOID,
    ID_ROCK,
    ID_BLOCK,
    //ID_TOWER,
    ID_SHAKE,
  };
  //Quaterniond oyaori[3];
  //Quaterniond hitoori[4];
  //Quaterniond nakaori[4];
  //Quaterniond kusuriori[4];
  //Quaterniond koori[4];

  PHSolidIf* fg_base[5][4];
  PHSolidIf* fg_obj[5][4];
  Vec3d fg_pos[5][4][2];
  Vec3d fg_pos_middle[5][4];
  Quaterniond fg_ori[5][4];
  PHSpringIf* fg_joint_vc[5][4];
  PHBallJointIf* fg_joint[5][3];

  //Controller controller;
  int64_t lastFrameID = 0;
  double RealToSpr = 1 / 15;
  //Vec3d Oya_position;
  //Quaterniond Oya_orientation;
  //PHSolidIf* soOyaBase;//Leapからの情報
  //PHSolidIf* soOyaObj;//実際に触る方
  //PHSpringIf* Oyajoint;
  PHSpringDesc Springdesc;
  PHBallJointDesc Balldesc[5][3];
  //Posed oyapose;

  //Vec3d Hito_position;
  //Quaterniond Hito_orientation;
  //PHSolidIf* soHitoBase;//Leapからの情報
  //PHSolidIf* soHitoObj;//実際に触る方
  //PHSpringIf* Hitojoint;
  //PHSpringDesc Hitodesc;

  PHSpringDesc oyahitodesc;



  ////Vec3d oya0;
  //Vec3d oya1p;
  //Vec3d oya2p;
  ////Vec3d oya3p;
  ////Vec3d hito0p;
  //Vec3d hito1p;
  //Vec3d hito2p;
  //Vec3d hito3p;
  //Vec3d naka0p;
  //Vec3d naka1p;
  //Vec3d naka2p;
  //Vec3d naka3p;
  //Vec3d kusuri0p;
  //Vec3d kusuri1p;
  //Vec3d kusuri2p;
  //Vec3d kusuri3p;

  //Vec3d ko0p;
  //Vec3d ko1p;
  //Vec3d ko2p;
  //Vec3d ko3p;

  ////Vec3d oya0n;
  //Vec3d oya1n;
  //Vec3d oya2n;
  //Vec3d oya3n;
  ////Vec3d hito0n;
  //Vec3d hito1n;
  //Vec3d hito2n;
  //Vec3d hito3n;
  //Vec3d naka0n;
  //Vec3d naka1n;
  //Vec3d naka2n;
  //Vec3d naka3n;
  //Vec3d kusuri0n;
  //Vec3d kusuri1n;
  //Vec3d kusuri2n;
  //Vec3d kusuri3n;
  //Vec3d ko0n;
  //Vec3d ko1n;
  //Vec3d ko2n;
  //Vec3d ko3n;


  PHSolidIf* jenga1;
  PHSolidIf* jenga2;
  PHSolidIf* jenga3;
  PHSolidIf* jenga4;
  PHSolidIf* jenga5;
  PHSolidIf* jenga6;
  PHSolidIf* jenga7;
  PHSolidIf* jenga8;
  PHSolidIf* jenga9;




  ////obj

  //PHSolidIf* sooya1;
  //PHSolidIf* sooya2;
  //PHSolidIf* sohito1;
  //PHSolidIf* sohito2;
  //PHSolidIf* sohito3;
  //PHSolidIf* sonaka0;
  //PHSolidIf* sonaka1;
  //PHSolidIf* sonaka2;
  //PHSolidIf* sonaka3;
  //PHSolidIf* sokusuri0;
  //PHSolidIf* sokusuri1;
  //PHSolidIf* sokusuri2;
  //PHSolidIf* sokusuri3;
  //PHSolidIf* soko0;
  //PHSolidIf* soko1;
  //PHSolidIf* soko2;
  //PHSolidIf* soko3;



  ////base

  //PHSolidIf* sooya1b;
  //PHSolidIf* sooya2b;
  //PHSolidIf* sohito1b;
  //PHSolidIf* sohito2b;
  //PHSolidIf* sohito3b;
  //PHSolidIf* sonaka0b;
  //PHSolidIf* sonaka1b;
  //PHSolidIf* sonaka2b;
  //PHSolidIf* sonaka3b;
  //PHSolidIf* sokusuri0b;
  //PHSolidIf* sokusuri1b;
  //PHSolidIf* sokusuri2b;
  //PHSolidIf* sokusuri3b;
  //PHSolidIf* soko0b;
  //PHSolidIf* soko1b;
  //PHSolidIf* soko2b;
  //PHSolidIf* soko3b;

  //PHSpringIf* oyajoint1;
  //PHSpringIf* oyajoint2;
  //PHSpringIf* hitojoint1;
  //PHSpringIf* hitojoint2;
  //PHSpringIf* hitojoint3;
  //PHSpringIf* nakajoint0;
  //PHSpringIf* nakajoint1;
  //PHSpringIf* nakajoint2;
  //PHSpringIf* nakajoint3;
  //PHSpringIf* kusurijoint0;
  //PHSpringIf* kusurijoint1;
  //PHSpringIf* kusurijoint2;
  //PHSpringIf* kusurijoint3;
  //PHSpringIf* kojoint0;
  //PHSpringIf* kojoint1;
  //PHSpringIf* kojoint2;
  //PHSpringIf* kojoint3;

  //PHSpringIf* oyahito;
  CDShapeIf* shapejenga;

  PHSolidIf* judge;

  Quaterniond x90;
  Quaterniond y180;

  clock_t start, end;
  CDBoxIf* shapeBoxBone;
  CDEllipsoidIf* shapenail;
  CDEllipsoidIf* shapepad;
  CDCapsuleIf* shapebone;

public:
  MyApp() {
    appName = "BoxStack";
    floorShakeAmplitude = 0.0;
    x90 = Quaterniond::Rot(Rad(90.0), 'x');
    y180 = Quaterniond::Rot(Rad(180.0), 'y');


    AddAction(MENU_MAIN, ID_BOX, "drop box");
    AddHotKey(MENU_MAIN, ID_BOX, 'b');
    AddAction(MENU_MAIN, ID_CAPSULE, "drop capsule");
    AddHotKey(MENU_MAIN, ID_CAPSULE, 'c');
    AddAction(MENU_MAIN, ID_ROUNDCONE, "drop round cone");
    AddHotKey(MENU_MAIN, ID_ROUNDCONE, 'r');
    AddAction(MENU_MAIN, ID_SPHERE, "drop sphere");
    AddHotKey(MENU_MAIN, ID_SPHERE, 's');
    AddAction(MENU_MAIN, ID_ELLIPSOID, "drop ellipsoid");
    AddHotKey(MENU_MAIN, ID_ELLIPSOID, 'E');
    AddAction(MENU_MAIN, ID_ROCK, "drop rock");
    AddHotKey(MENU_MAIN, ID_ROCK, 'd');
    AddAction(MENU_MAIN, ID_BLOCK, "drop block");
    AddHotKey(MENU_MAIN, ID_BLOCK, 'e');
    //AddAction(MENU_MAIN, ID_TOWER, "drop tower");
    //AddHotKey(MENU_MAIN, ID_TOWER, 't');
    AddAction(MENU_MAIN, ID_SHAKE, "shake floor");
    AddHotKey(MENU_MAIN, ID_SHAKE, 'f');

  }
  ~MyApp() {}

  PHSolidIf* CreateThin() {
    PHSolidIf* thin = GetPHScene()->CreateSolid();
    CDBoxDesc bd;
    bd.boxsize = Vec3d(20.0, 0.5, 20.0);
    CDBoxIf* shapethin;
    shapethin = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
    thin->AddShape(shapethin);
    return thin;
  }

  PHSolidIf* CreateJenga() {
    PHSolidIf* jenga = GetPHScene()->CreateSolid();
    CDBoxDesc bd;
    bd.boxsize = Vec3d(2.8, 2.8, 8.5);
    shapejenga = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
    shapejenga->SetStaticFriction(100.0f);
    shapejenga->SetDynamicFriction(100.0f);
    //shapejenga->SetDensity(100.0);
    jenga->AddShape(shapejenga);
    jenga->SetDynamical(false);
    return jenga;
  }
  PHSolidIf* CreateJenga2() {
    PHSolidIf* jenga = GetPHScene()->CreateSolid();
    CDBoxDesc bd;
    bd.boxsize = Vec3d(2.9, 2.9, 8.7);
    shapejenga = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
    shapejenga->SetStaticFriction(10.0f);
    shapejenga->SetDynamicFriction(10.0f);
    //shapejenga->SetDensity(100.0);
    jenga->AddShape(shapejenga);

    return jenga;
  }
  /// 床の作成
  PHSolidIf* CreateFloor(bool bWall) {

    PHSolidIf* soFloor = GetPHScene()->CreateSolid();
    soFloor->SetName("soFloor");
    soFloor->SetDynamical(false);

    soFloor->AddShape(shapeFloor);

    soFloor->SetShapePose(0, Posed::Trn(22.0, -shapeFloor->GetBoxSize().y / 2, 0));
    if (bWall) {
      soFloor->AddShape(shapeWallZ);
      soFloor->AddShape(shapeWallX);
      soFloor->AddShape(shapeWallZ);
      soFloor->AddShape(shapeWallX);
      double y = shapeWallZ->GetBoxSize().y / 2 - shapeFloor->GetBoxSize().y;
      soFloor->SetShapePose(1, Posed::Trn(-(shapeFloor->GetBoxSize().x + shapeWallZ->GetBoxSize().x) / 2, y, 0));
      soFloor->SetShapePose(2, Posed::Trn(0, y, -(shapeFloor->GetBoxSize().z + shapeWallX->GetBoxSize().z) / 2));
      soFloor->SetShapePose(3, Posed::Trn((shapeFloor->GetBoxSize().x + shapeWallZ->GetBoxSize().x) / 2, y, 0));
      soFloor->SetShapePose(4, Posed::Trn(0, y, (shapeFloor->GetBoxSize().z + shapeWallX->GetBoxSize().z) / 2));
    }
    soFloor->AddShape(shapeFloor);
    soFloor->AddShape(shapeFloor);
    soFloor->AddShape(shapeFloor);
    soFloor->SetShapePose(5, Posed::Trn(-22.0, -shapeFloor->GetBoxSize().y / 2, 0));
    soFloor->SetShapePose(6, Posed::Trn(0, -shapeFloor->GetBoxSize().y / 2, -22.0));
    soFloor->SetShapePose(7, Posed::Trn(0, -shapeFloor->GetBoxSize().y / 2, 42.3));

    GetFWScene()->SetSolidMaterial(GRRenderIf::GRAY, soFloor);
    soFloor->CompInertia();

    return soFloor;
  }

  //指先
  PHSolidIf* CreateBone(float width) {

    float BlenderToSpr = width / 2.48 / 15;
    printf("width=%f,BlenderToSpr=%f", width, BlenderToSpr);
    PHSolidIf* soBone = GetPHScene()->CreateSolid();

    CDEllipsoidDesc nail;
    //nail.radius = Vec3d(1.69, 0.179, 1.12)*(double)BlenderToSpr;
    nail.radius = Vec3d(2.5, 0.179, 1.12) * (double)BlenderToSpr;
    shapenail = GetSdk()->GetPHSdk()->CreateShape(nail)->Cast();
    shapenail->SetDensity(1.0);
    shapenail->SetStaticFriction(1.0);
    shapenail->SetDynamicFriction(1.0);

    CDEllipsoidDesc pad;
    pad.radius = Vec3d(2.79, 1.26, 1.5) * (double)BlenderToSpr;
    shapepad = GetSdk()->GetPHSdk()->CreateShape(pad)->Cast();
    shapepad->SetDensity(1.0);
    shapepad->SetStaticFriction(1.0);
    shapepad->SetDynamicFriction(1.0);

    CDCapsuleDesc bone;
    bone.radius = 1.24 * (double)BlenderToSpr;
    bone.length = (2 - 1.24) * 2 * (double)BlenderToSpr;
    shapebone = GetSdk()->GetPHSdk()->CreateShape(bone)->Cast();
    shapebone->SetDensity(1.0);
    //shapebone->SetStaticFriction(2.5);
    //shapebone->SetDynamicFriction(2.5);

    soBone->AddShape(shapenail);
    soBone->AddShape(shapepad);
    soBone->AddShape(shapebone);


    soBone->SetShapePose(0, Posed(Vec3d(0.0, 0.31 * 2, -0.67 * 2) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90), 'y')));
    soBone->SetShapePose(1, Posed(Vec3d(0.0, 0, 0) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90.0), 'y')));
    soBone->SetShapePose(2, Posed(Vec3d(0.0, 0, 0.38 * 2) * (double)BlenderToSpr, Quaterniond::Rot(Rad(0.0), 'y')));//180かも

    //soBone->CompInertia();
    soBone->SetMass(1.0f);
    soBone->SetGravity(false);
    return soBone;

  }

  //指先(爪なし)
  PHSolidIf* CreateBoneNo(float width) {

    float BlenderToSpr = width / 2.48 / 15;
    printf("width=%f,BlenderToSpr=%f", width, BlenderToSpr);
    PHSolidIf* soBone = GetPHScene()->CreateSolid();

    //CDEllipsoidDesc nail;
    ////nail.radius = Vec3d(1.69, 0.179, 1.12)*(double)BlenderToSpr;
    //nail.radius = Vec3d(2.0, 0.179, 1.12) * (double)BlenderToSpr;
    //shapenail = GetSdk()->GetPHSdk()->CreateShape(nail)->Cast();
    //shapenail->SetDensity(1.0);
    //shapenail->SetStaticFriction(10.0);
    //shapenail->SetDynamicFriction(10.0);

    CDEllipsoidDesc pad;
    pad.radius = Vec3d(2.79, 1.26, 1.5) * (double)BlenderToSpr;
    shapepad = GetSdk()->GetPHSdk()->CreateShape(pad)->Cast();
    shapepad->SetDensity(1.0f);
    shapepad->SetStaticFriction(1.0f);
    shapepad->SetDynamicFriction(1.0f);

    CDCapsuleDesc bone;
    bone.radius = 1.24 * (double)BlenderToSpr;
    bone.length = (2 - 1.24) * 2 * (double)BlenderToSpr;
    shapebone = GetSdk()->GetPHSdk()->CreateShape(bone)->Cast();
    shapebone->SetDensity(1.0f);
    //shapebone->SetStaticFriction(2.5);
    //shapebone->SetDynamicFriction(2.5);

    //soBone->AddShape(shapenail);
    soBone->AddShape(shapepad);
    soBone->AddShape(shapebone);


    //soBone->SetShapePose(0, Posed(Vec3d(0, 0.31 * 2, -0.67 * 2) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90), 'y')));
    soBone->SetShapePose(0, Posed(Vec3d(0, 0, 0) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90), 'y')));
    soBone->SetShapePose(1, Posed(Vec3d(0, 0, 0.38 * 2) * (double)BlenderToSpr, Quaterniond::Rot(Rad(0), 'y')));//180かも

    //soBone->CompInertia();
    soBone->SetMass(1.0f);
    //soBone->SetGravity(false);
    return soBone;

  }

  //fg_base用のsolid
  PHSolidIf* CreateBone2() {
    PHSolidIf* soBone = GetPHScene()->CreateSolid();
    CDSphereDesc sd;
    sd.radius = 0.3f;

    shapeSphere = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();

    soBone->SetDynamical(false);


    soBone->AddShape(shapeSphere);


    return soBone;
  }
  //fg_obj用のsolid
  PHSolidIf* CreateBone3(Vec3d p, Vec3d n) {
    PHSolidIf* soBone = GetPHScene()->CreateSolid();
    CDCapsuleDesc cd;
    cd.radius = 1.0f;
    cd.length = (float)sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y) + (p.z - n.z) * (p.z - n.z));

    shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
    shapeCapsule->SetDensity(0.1f);

    soBone->AddShape(shapeCapsule);
    soBone->SetGravity(false);
    //soBone->SetDynamical(false);
    //soBone->SetDynamical(false);
    soBone->CompInertia();

    return soBone;
  }
  PHSolidIf* CreateBone4() {
    //立方体
    PHSolidIf* soBone = GetPHScene()->CreateSolid();
    CDBoxIf* shapeBone;
    CDBoxDesc sd;
    sd.boxsize = Vec3d(1.5, 1.5, 1.5);

    shapeBone = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();
    shapeBone->SetStaticFriction(1.0);
    shapeBone->SetDynamicFriction(1.0);

    //soBone->SetDynamical(true);
    //soBone->SetName("soBone");

    soBone->AddShape(shapeBone);

    soBone->SetMass(1.0);
    //soBone->SetGravity(false);
    return soBone;
  }
  PHSolidIf* CreateBone5() {
    //球
    PHSolidIf* soBone = GetPHScene()->CreateSolid();
    CDSphereIf* Bone5;
    CDSphereDesc sd;
    sd.radius = 1.0;

    Bone5 = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();
    Bone5->SetStaticFriction(1.0);
    Bone5->SetDynamicFriction(1.0);

    soBone->SetDynamical(false);
    //soBone->SetName("soBone");

    soBone->AddShape(Bone5);
    //soBone->SetMass(1.0);
    //soBone->SetGravity(false);
    //soBone->SetFramePosition(Vec3d(0, 0, 0));
    //soBone->CompInertia();

    return soBone;
  }


  Vec3d vecvec3(LEAP_VECTOR vec, LEAP_VECTOR vec2) {
    Vec3d vec3;
    double x;
    double y;
    double z;
    x = (double)((double)vec.x + (double)vec2.x);
    y = (double)((double)vec.y + (double)vec2.y);
    z = (double)((double)vec.z + (double)vec2.z);
    x /= 2;
    y /= 2;
    z /= 2;

    vec3.x = -x / 10.0;
    vec3.y = 25.0 - z / 10.0;
    vec3.z = 30.0 - y / 10.0;


    return vec3;
  }
  Vec3d vecvec3_2(LEAP_VECTOR vec) {
    Vec3d vec3;
    vec3.x = (double)(vec.x / 10.0);
    vec3.y = (double)(vec.y / 10.0);
    vec3.z = (double)(vec.z / 10.0);

    return vec3;
  }
  Quaterniond quaquad(LEAP_QUATERNION qua) {
    Quaterniond quad;
    quad.x = (double)qua.x;
    quad.y = (double)qua.y;
    quad.z = (double)qua.z;
    quad.w = (double)qua.w;
    return quad;
  }

  virtual void BuildScene() {

    soFloor = CreateFloor(true);

    //soPalm = CreateBone();

    GetFWScene()->EnableRenderAxis(false, false, false);


    //指先の作成

    //親指

    printf("手をかざしてお待ちください。\n");
 
    LEAP_TRACKING_EVENT* frame = GetFrame();
    LEAP_HAND* hand = &frame->pHands[0];
    //printf("hand is detected\n");

    float tip_width[5] = {0.0};


    //float thunb_width;
    //float index_width;
    //float middle_width;
    //float ring_width;
    //float pinky_width;

    //thunb_width = 20.0;
    //index_width = 20.0;
    //middle_width = 20.0;
    //ring_width = 20.0;
    //pinky_width = 20.0;

    for (int i = 0; i < 5; i++) {
      tip_width[i] = hand->digits[i].distal.width;
      tip_width[i] *= 1.5;
    }


    //thunb_width = hand->thumb.distal.width;
    //index_width = hand->index.distal.width;
    //middle_width = hand->middle.distal.width;
    //ring_width = hand->ring.distal.width;
    //pinky_width = hand->pinky.distal.width;

    //oya1n = vecvec3_2(hand->thumb.intermediate.next_joint);
    //oya1p = vecvec3_2(hand->thumb.intermediate.prev_joint);
    //oya2n = vecvec3_2(hand->thumb.proximal.next_joint);
    //oya2p = vecvec3_2(hand->thumb.proximal.prev_joint);
    //hito1n = vecvec3_2(hand->index.intermediate.next_joint);
    //hito1p = vecvec3_2(hand->index.intermediate.prev_joint);
    //hito2n = vecvec3_2(hand->index.proximal.next_joint);
    //hito2p = vecvec3_2(hand->index.proximal.prev_joint);
    //hito3n = vecvec3_2(hand->index.metacarpal.next_joint);
    //hito3p = vecvec3_2(hand->index.metacarpal.prev_joint);
    //naka0n = vecvec3_2(hand->middle.distal.next_joint);
    //naka0p = vecvec3_2(hand->middle.distal.prev_joint);
    //naka1n = vecvec3_2(hand->middle.intermediate.next_joint);
    //naka1p = vecvec3_2(hand->middle.intermediate.prev_joint);
    //naka2n = vecvec3_2(hand->middle.proximal.next_joint);
    //naka2p = vecvec3_2(hand->middle.proximal.prev_joint);
    //naka3n = vecvec3_2(hand->middle.metacarpal.next_joint);
    //naka3p = vecvec3_2(hand->middle.metacarpal.prev_joint);
    //kusuri0n = vecvec3_2(hand->ring.distal.next_joint);
    //kusuri0p = vecvec3_2(hand->ring.distal.prev_joint);
    //kusuri1n = vecvec3_2(hand->ring.intermediate.next_joint);
    //kusuri1p = vecvec3_2(hand->ring.intermediate.prev_joint);
    //kusuri2n = vecvec3_2(hand->ring.proximal.next_joint);
    //kusuri2p = vecvec3_2(hand->ring.proximal.prev_joint);
    //kusuri3n = vecvec3_2(hand->ring.metacarpal.next_joint);
    //kusuri3p = vecvec3_2(hand->ring.metacarpal.prev_joint);
    //ko0n = vecvec3_2(hand->pinky.distal.next_joint);
    //ko0p = vecvec3_2(hand->pinky.distal.prev_joint);
    //ko1n = vecvec3_2(hand->pinky.intermediate.next_joint);
    //ko1p = vecvec3_2(hand->pinky.intermediate.prev_joint);
    //ko2n = vecvec3_2(hand->pinky.proximal.next_joint);
    //ko2p = vecvec3_2(hand->pinky.proximal.prev_joint);
    //ko3n = vecvec3_2(hand->pinky.metacarpal.next_joint);
    //ko3p = vecvec3_2(hand->pinky.metacarpal.prev_joint);




    Springdesc.spring = Vec3d(15000.0, 15000.0, 1500.0);
    Springdesc.damper = Vec3d(150.0, 150.0, 150.0);
    Springdesc.springOri = 10000.0;
    Springdesc.damperOri = 100.0;

    for(int i=0;i<5;i++){
      for (int j = 0; j < 4; j++) {
        if (!(i == 0 && j == 0)) {
          fg_pos[i][j][0] = vecvec3_2(hand->digits[i].bones[j].prev_joint);
          fg_pos[i][j][1] = vecvec3_2(hand->digits[i].bones[j].next_joint);
        }
      }
    }

    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 4; j++) {
        if (!(i == 0 && j == 0)) {
          
          fg_base[i][j] = CreateBone2();
          GetFWScene()->SetSolidMaterial(GRRenderIf::RED, fg_base[i][j]);
          if (j == 3) {//指先
            fg_obj[i][j] = CreateBone(tip_width[i]);
          }
          else {//指の骨
            fg_obj[i][j] = CreateBone3(fg_pos[i][j][0],fg_pos[i][j][1]);
          }
          
          GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj[i][j]);

          fg_joint_vc[i][j] = GetPHScene()->CreateJoint(fg_base[i][j], fg_obj[i][j], Springdesc)->Cast();
        }
      }
    }
    //for (int i = 0; i < 5; i++) {
    //    for (int j = 0; j < 3; j++) {
    //        if (!(i == 0 && j == 0)) {
    //            Balldesc[i][j].poseSocket.Pos() = (fg_pos[i][j][1] - fg_pos[i][j][0]) / 2.0;
    //            Balldesc[i][j].posePlug.Pos() = (fg_pos[i][j + 1][0] - fg_pos[i][j + 1][1]) / 2.0;
    //            fg_joint[i][j] = GetPHScene()->CreateJoint(fg_obj[i][j], fg_obj[i][j + 1], Balldesc[i][j])->Cast();
    //        }
    //    }
    //}



    //soOyaBase = CreateBone2();
    //soOyaBase->SetDynamical(false);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::RED, soOyaBase);
    //printf("thumb_width=%f\n", thunb_width);
    ////soOyaObj = CreateBone(thunb_width);
    //soOyaObj = CreateBone(thunb_width);

    ////soOyaObj->SetDynamical(true);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, soOyaObj);
    ////soOyaObj->SetMass(1);
    //Springdesc.spring = Vec3d(1500, 1500, 1500);
    //Springdesc.damper = Vec3d(15, 15, 15);
    //Springdesc.springOri = 10000.0;
    //Springdesc.damperOri = 1000.0;
    //Oyajoint = GetPHScene()->CreateJoint(soOyaBase, soOyaObj, Springdesc)->Cast();

    ////人差し指
    //soHitoBase = CreateBone2();
    //soHitoBase->SetDynamical(false);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::RED, soHitoBase);
    ////soHitoObj = CreateBone(index_width);
    //soHitoObj = CreateBone(index_width);

    //GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, soHitoObj);
    ////soHitoObj->SetMass(1);
    //Hitojoint = GetPHScene()->CreateJoint(soHitoBase, soHitoObj, Springdesc)->Cast();


    //oyahitodesc.spring = Vec3d(200, 200, 200);
    //oyahitodesc.damper = Vec3d(15, 15, 15);
    //oyahitodesc.springOri = 100;
    //oyahitodesc.damperOri = 3;
    //oyahito= GetPHScene()->CreateJoint(soOyaObj, soHitoObj, oyahitodesc)->Cast();


    //
    ////指の作成

    ////obj


    //sooya1 = CreateBone3(oya1n,oya1p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya1);   
    //sooya2 = CreateBone3(oya2n,oya2p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya2);    


    //sohito1 = CreateBone3(hito1n,hito1p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito1);
    //sohito2 = CreateBone3(hito2n,hito2p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito2);
    //sohito3 = CreateBone3(hito3n,hito3p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito3);

    //sonaka0 = CreateBone5();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka0);
    //sonaka1 = CreateBone3(naka1n,naka1p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka1);
    //sonaka2 = CreateBone3(naka2n,naka2p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka2);
    //sonaka3 = CreateBone3(naka3n,naka3p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka3);

    //sokusuri0 = CreateBone5();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri0);
    //sokusuri1 = CreateBone3(kusuri1n,kusuri1p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri1);
    //sokusuri2 = CreateBone3(kusuri2n,kusuri2p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri2);
    //sokusuri3 = CreateBone3(kusuri3n,kusuri3p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri3);

    //soko0 = CreateBone5();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko0);
    //soko1 = CreateBone3(ko1n,ko1p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko1);
    //soko2 = CreateBone3(ko2n,ko2p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko2);
    //soko3 = CreateBone3(ko3n,ko3p);
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko3);


    //////base


    //sooya1b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya1b);
    //sooya2b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya2b);

    //sohito1b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito1b);
    //sohito2b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito2b);
    //sohito3b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito3);

    //sonaka0b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka0b);
    //sonaka1b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka1b);
    //sonaka2b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka2b);
    //sonaka3b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka3b);

    //sokusuri0b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri0b);
    //sokusuri1b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri1b);
    //sokusuri2b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri2b);
    //sokusuri3b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri3b);

    //soko0b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko0b);
    //soko1b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko1b);
    //soko2b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko2b);
    //soko3b = CreateBone2();
    //GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko3b);


    ////joint
    //oyajoint1 = GetPHScene()->CreateJoint(sooya1b, sooya1, Springdesc)->Cast();
    //oyajoint2 = GetPHScene()->CreateJoint(sooya2b, sooya2, Springdesc)->Cast();
    //hitojoint1 = GetPHScene()->CreateJoint(sohito1b, sohito1, Springdesc)->Cast();
    //hitojoint2 = GetPHScene()->CreateJoint(sohito2b, sohito2, Springdesc)->Cast();
    //hitojoint3 = GetPHScene()->CreateJoint(sohito3b, sohito3, Springdesc)->Cast();
    //nakajoint0 = GetPHScene()->CreateJoint(sonaka0b, sonaka0, Springdesc)->Cast();
    //nakajoint1 = GetPHScene()->CreateJoint(sonaka1b, sonaka1, Springdesc)->Cast();
    //nakajoint2 = GetPHScene()->CreateJoint(sonaka2b, sonaka2, Springdesc)->Cast();
    //nakajoint3 = GetPHScene()->CreateJoint(sonaka3b, sonaka3, Springdesc)->Cast();
    //kusurijoint0 = GetPHScene()->CreateJoint(sokusuri0b, sokusuri0, Springdesc)->Cast();
    //kusurijoint1 = GetPHScene()->CreateJoint(sokusuri1b, sokusuri1, Springdesc)->Cast();
    //kusurijoint2 = GetPHScene()->CreateJoint(sokusuri2b, sokusuri2, Springdesc)->Cast();
    //kusurijoint3 = GetPHScene()->CreateJoint(sokusuri3b, sokusuri3, Springdesc)->Cast();
    //kojoint0 = GetPHScene()->CreateJoint(soko0b, soko0, Springdesc)->Cast();
    //kojoint1 = GetPHScene()->CreateJoint(soko1b, soko1, Springdesc)->Cast();
    //kojoint2 = GetPHScene()->CreateJoint(soko2b, soko2, Springdesc)->Cast();
    //kojoint3 = GetPHScene()->CreateJoint(soko3b, soko3, Springdesc)->Cast();



    //GetPHScene()->SetContactMode(soOyaBase,PHSceneDesc::MODE_NONE);

  }

  // タイマコールバック関数．タイマ周期で呼ばれる
  virtual void OnStep() {
    // GetSdk()->SaveScene("test.spr", NULL, FIFileSprIf::GetIfInfoStatic());

    SampleApp::OnStep();
    //printf("a");

    /*GetPHScene()->SetContactMode(sooya1b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sooya2b, PHSceneDesc::MODE_NONE);

    GetPHScene()->SetContactMode(sohito1b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sohito2b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sohito3b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sonaka0b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sonaka1b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sonaka2b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sonaka3b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sokusuri0b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sokusuri1b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sokusuri2b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sokusuri3b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(soko0b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(soko1b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(soko2b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(soko3b, PHSceneDesc::MODE_NONE);

    GetPHScene()->SetContactMode(soOyaBase, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(soHitoBase, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sonaka0b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(sokusuri0b, PHSceneDesc::MODE_NONE);
    GetPHScene()->SetContactMode(soko0b, PHSceneDesc::MODE_NONE);*/


    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            if (!(i == 0 && j == 0)) {
                GetPHScene()->SetContactMode(fg_base[i][j], PHSceneDesc::MODE_NONE);
                GetPHScene()->SetContactMode(fg_obj[i][j], soFloor, PHSceneDesc::MODE_NONE);
            }
        }
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4 - 1; j++) {
            if (!(i == 0 && j == 0)) GetPHScene()->SetContactMode(fg_obj[i][j], fg_obj[i][j + 1], PHSceneDesc::MODE_NONE);
        }
    }
    for (int i = 1; i < 5 - 1; i++) {
        GetPHScene()->SetContactMode(fg_obj[i][0], fg_obj[i + 1][0], PHSceneDesc::MODE_NONE);
    }

    // 床を揺らす
    if (soFloor && floorShakeAmplitude) {
      double time = GetFWScene()->GetPHScene()->GetCount() * GetFWScene()->GetPHScene()->GetTimeStep();
      double omega = 2.0 * M_PI;
      soFloor->SetFramePosition(Vec3d(floorShakeAmplitude * sin(time * omega), 0, 0));
      soFloor->SetVelocity(Vec3d(floorShakeAmplitude * omega * cos(time * omega), 0, 0));
    }



    LEAP_TRACKING_EVENT* frame = GetFrame();

    if (frame && (frame->tracking_frame_id > lastFrameID)) {
      lastFrameID = frame->tracking_frame_id;
      //printf("Frame %lli with %i hands.\n", (long long int)frame->tracking_frame_id, frame->nHands);
      for (uint32_t h = 0; h < frame->nHands; h++) {
        LEAP_HAND* hand = &frame->pHands[0];

        for (int i = 0; i < 5; i++) {
          for (int j = 0; j < 4; j++) {
            if (!(i == 0 && j == 0)) {
              fg_pos_middle[i][j] = vecvec3(hand->digits[i].bones[j].prev_joint, hand->digits[i].bones[j].next_joint);
              fg_ori[i][j] = quaquad(hand->digits[i].bones[j].rotation);
              fg_ori[i][j] = x90 * fg_ori[i][j];
              fg_ori[i][j] = y180 * fg_ori[i][j];
              fg_base[i][j]->SetFramePosition(fg_pos_middle[i][j]);
              fg_base[i][j]->SetOrientation(fg_ori[i][j]);
            }
          }
        }

        /*Oya_position = vecvec3(hand->thumb.distal.prev_joint, hand->thumb.distal.next_joint);
        Oya_orientation = quaquad(hand->thumb.distal.rotation);
        Hito_position = vecvec3(hand->index.distal.prev_joint, hand->index.distal.next_joint);
        Hito_orientation = quaquad(hand->index.distal.rotation);


        Oya_orientation = x90 * Oya_orientation;
        Oya_orientation = y180 * Oya_orientation;


        soOyaBase->SetFramePosition(Oya_position);
        soOyaBase->SetOrientation(Oya_orientation);



        Hito_orientation = x90 * Hito_orientation;
        Hito_orientation = y180 * Hito_orientation;

        soHitoBase->SetFramePosition(Hito_position);
        soHitoBase->SetOrientation(Hito_orientation);

        oya2p = vecvec3(hand->thumb.proximal.prev_joint, hand->thumb.proximal.next_joint);
        oya1p = vecvec3(hand->thumb.intermediate.prev_joint, hand->thumb.intermediate.next_joint);

        hito3p = vecvec3(hand->index.metacarpal.prev_joint, hand->index.metacarpal.next_joint);
        hito2p = vecvec3(hand->index.proximal.prev_joint, hand->index.proximal.next_joint);
        hito1p = vecvec3(hand->index.intermediate.prev_joint, hand->index.intermediate.next_joint);

        naka3p = vecvec3(hand->middle.metacarpal.prev_joint, hand->middle.metacarpal.next_joint);
        naka0p = vecvec3(hand->middle.distal.prev_joint, hand->middle.distal.next_joint);
        naka2p = vecvec3(hand->middle.proximal.prev_joint, hand->middle.proximal.next_joint);
        naka1p = vecvec3(hand->middle.intermediate.prev_joint, hand->middle.intermediate.next_joint);

        kusuri3p = vecvec3(hand->ring.metacarpal.prev_joint, hand->ring.metacarpal.next_joint);
        kusuri0p = vecvec3(hand->ring.distal.prev_joint, hand->ring.distal.next_joint);
        kusuri2p = vecvec3(hand->ring.proximal.prev_joint, hand->ring.proximal.next_joint);
        kusuri1p = vecvec3(hand->ring.intermediate.prev_joint, hand->ring.intermediate.next_joint);

        ko3p = vecvec3(hand->pinky.metacarpal.prev_joint, hand->pinky.metacarpal.next_joint);
        ko0p = vecvec3(hand->pinky.distal.prev_joint, hand->pinky.distal.next_joint);
        ko2p = vecvec3(hand->pinky.proximal.prev_joint, hand->pinky.proximal.next_joint);
        ko1p = vecvec3(hand->pinky.intermediate.prev_joint, hand->pinky.intermediate.next_joint);



        sooya1b->SetFramePosition(oya1p);
        sooya2b->SetFramePosition(oya2p);
        sohito1b->SetFramePosition(hito1p);
        sohito2b->SetFramePosition(hito2p);
        sohito3b->SetFramePosition(hito3p);
        sonaka0b->SetFramePosition(naka0p);
        sonaka1b->SetFramePosition(naka1p);
        sonaka2b->SetFramePosition(naka2p);
        sonaka3b->SetFramePosition(naka3p);
        sokusuri0b->SetFramePosition(kusuri0p);
        sokusuri1b->SetFramePosition(kusuri1p);
        sokusuri2b->SetFramePosition(kusuri2p);
        sokusuri3b->SetFramePosition(kusuri3p);
        soko0b->SetFramePosition(ko0p);
        soko1b->SetFramePosition(ko1p);
        soko2b->SetFramePosition(ko2p);
        soko3b->SetFramePosition(ko3p);

        for (int i = 0; i < 3; i++) {
          oyaori[i] = quaquad(hand->thumb.bones[i].rotation);
          oyaori[i] = x90 * oyaori[i];
          oyaori[i] = y180 * oyaori[i];
        }*/
      }
    }

    //ここでフレーム処理
    //ConnectionCallbacks.on_frame = &OnFrame;


    //const Frame frame = controller.frame();

    //
    //HandList hands = frame.hands();
    ////FingerList fingers = frame.fingers();

    //for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    //  // Get the first hand
    //  const Hand hand = *hl;
    //  spr_palm[0] = double(hand.palmPosition()[0]/10);
    //  spr_palm[1] = double(hand.palmPosition()[1]/10);
    //  spr_palm[2] = double(hand.palmPosition()[2]/10);

    //  cout << hand.direction() << hand.palmNormal() << endl;
    //}

    //
    //for (auto finger : frame.fingers()) {
    //  if (finger.type() == Leap::Finger::Type::TYPE_THUMB) {
    //    for (int t = Leap::Bone::TYPE_METACARPAL;
    //      t <= (int)Leap::Bone::TYPE_DISTAL; ++t) {
    //      // 末節骨(指先の骨)
    //      if (t == 3) {
    //        auto bone = finger.bone((Leap::Bone::Type)t);
    //        
    //        spr_oya3[0] = double(bone.prevJoint()[0] / 10);
    //        spr_oya3[1] = double(bone.prevJoint()[1] / 10);
    //        spr_oya3[2] = double(bone.prevJoint()[2] / 10);
    //      }

    //    }
    //  }
    // 
    //  
    //  if (finger.type() == Leap::Finger::Type::TYPE_INDEX) {
    //    for (int t = Leap::Bone::TYPE_METACARPAL;
    //      t <= (int)Leap::Bone::TYPE_DISTAL; ++t) {
    //      // 末節骨(指先の骨)
    //      if (t == 3) {
    //        auto bone = finger.bone((Leap::Bone::Type)t);
    //        spr_hito3[0] = double(bone.prevJoint()[0] / 10);
    //        spr_hito3[1] = double(bone.prevJoint()[1] / 10);
    //        spr_hito3[2] = double(bone.prevJoint()[2] / 10);
    //      }
    //    }
    //  }
    //  
    //}
    //GetFWScene()->SetSolidMaterial(GRRenderIf::RED, soPalm);




  }

  // 描画関数．描画要求が来たときに呼ばれる
  virtual void OnDraw(GRRenderIf* render) {
    SampleApp::OnDraw(render);

    std::ostringstream sstr;
    sstr << "NObj = " << GetFWScene()->GetPHScene()->NSolids();
    render->DrawFont(Vec2f(-21, 23), sstr.str());
  }

  virtual void OnAction(int menu, int id) {
    if (menu == MENU_MAIN) {
      Vec3d v, w(0.0, 0.0, 0.2), p(0.5, 10, 0.0);
      static Quaterniond q = Quaterniond::Rot(Rad(0.0), 'y');
      q = Quaterniond::Rot(Rad(90), 'y') * q;

      if (id == ID_BOX) {
        Drop(SHAPE_BOX, GRRenderIf::RED, v, w, p, q);
        message = "box dropped.";
        start = clock();
      }
      if (id == ID_CAPSULE) {
        Drop(SHAPE_CAPSULE, GRRenderIf::GREEN, v, w, p, q);
        message = "capsule dropped.";
        end = clock();
        cout << ((float)end - start) / CLOCKS_PER_SEC << endl;
      }
      if (id == ID_ROUNDCONE) {
        //Drop(SHAPE_ROUNDCONE, GRRenderIf::BLUE, v, w, p, q);
        //message = "round cone dropped.";
        jenga1 = CreateJenga();
        jenga1->SetFramePosition(Vec3d(1.5, 1.5, 4.5));
        jenga2 = CreateJenga();

        jenga2->SetFramePosition(Vec3d(4.5, 1.5, 4.5));
        jenga3 = CreateJenga();

        jenga3->SetFramePosition(Vec3d(7.5, 1.5, 4.5));
        jenga4 = CreateJenga();

        jenga4->SetFramePosition(Vec3d(4.5, 4.5, 1.5));
        jenga4->SetOrientation(Quaterniond::Rot(Rad(90.0), 'y'));
        jenga6 = CreateJenga();

        jenga6->SetFramePosition(Vec3d(4.5, 4.5, 7.5));
        jenga6->SetOrientation(Quaterniond::Rot(Rad(90.0), 'y'));
        jenga7 = CreateJenga();

        jenga7->SetFramePosition(Vec3d(1.5, 7.5, 4.5));
        jenga8 = CreateJenga();

        jenga8->SetFramePosition(Vec3d(4.5, 7.5, 4.5));
        jenga9 = CreateJenga();

        jenga9->SetFramePosition(Vec3d(7.5, 7.5, 4.5));
      }
      if (id == ID_SPHERE) {
        //Drop(SHAPE_SPHERE, GRRenderIf::YELLOW, v, w, p, q);
        //message = "sphere dropped.";
        jenga5 = CreateJenga2();
        jenga5->SetMass(1.0);
        jenga5->SetFramePosition(Vec3d(4.8, 4.5, 4.5));
        jenga5->SetOrientation(Quaterniond::Rot(Rad(90.0), 'y'));
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, jenga5);
      }
      if (id == ID_ELLIPSOID) {
        Drop(SHAPE_ELLIPSOID, GRRenderIf::LIGHTGREEN, v, w, p, q);
        message = "sphere dropped.";
      }
      if (id == ID_ROCK) {
        //Drop(SHAPE_ROCK, GRRenderIf::ORANGE, v, w, p, q);
        //message = "random polyhedron dropped.";
        CreateThin();
      }
      if (id == ID_BLOCK) {
        //Drop(SHAPE_BLOCK, GRRenderIf::CYAN, v, w, p, q);
        //message = "composite block dropped.";
        jenga1 = CreateJenga();
        jenga1->SetFramePosition(Vec3d(4.5, 1.5, 1.5));
        jenga1->SetOrientation(Quaterniond::Rot(Rad(90.0), 'y'));
        jenga2 = CreateJenga();
        jenga2->SetFramePosition(Vec3d(4.5, 1.5, 7.5));
        jenga2->SetOrientation(Quaterniond::Rot(Rad(90.0), 'y'));
        jenga3 = CreateJenga();
        jenga3->SetFramePosition(Vec3d(10.5, 1.5, 4.5));
        jenga4 = CreateJenga();
        jenga4->SetFramePosition(Vec3d(-1.5, 1.5, 4.5));
      }

      if (id == ID_SHAKE) {
        //std::cout << "F: shake floor." << std::endl;
        //if (floorShakeAmplitude == 0.0) {
        //  floorShakeAmplitude = 2.5;
        //  message = "floor shaken.";
        //}
        //else {
        //  floorShakeAmplitude = 0;
        //  soFloor->SetFramePosition(Vec3d(0, 0, 0));
        //  soFloor->SetVelocity(Vec3d(0, 0, 0));
        //  message = "floor stopped.";
        //}
        jenga5 = CreateJenga2();
        jenga5->SetMass(1.0);
        jenga5->SetFramePosition(Vec3d(4.5, 1.5, 4.5));
        jenga5->SetOrientation(Quaterniond::Rot(Rad(90.0), 'y'));
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, jenga5);
      }
    }
    SampleApp::OnAction(menu, id);
  }

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

MyApp app;




int main(int argc, char** argv) {
  ConnectionCallbacks.on_connection = &OnConnect;
  ConnectionCallbacks.on_device_found = &OnDevice;
  //ConnectionCallbacks.on_frame = &OnFrame;
  //ConnectionCallbacks.on_image = &OnImage;
  ConnectionCallbacks.on_point_mapping_change = &OnPointMappingChange;
  ConnectionCallbacks.on_log_message = &OnLogMessage;
  //ConnectionCallbacks.on_head_pose = &OnHeadPose;

  //connectionHandle = OpenConnection();
  //{
  //  LEAP_ALLOCATOR allocator = { allocate, deallocate, NULL };
  //  LeapSetAllocator(*connectionHandle, &allocator);
  //}

  //LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints, 0);

  connectionHandle = OpenConnection();
  {
    LEAP_ALLOCATOR allocator = { allocate, deallocate, NULL };
    LeapSetAllocator(*connectionHandle, &allocator);
  }
  LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_OptimizeHMD, 0);
  while (!IsConnected)
    millisleep(100); //wait a bit to let the connection complete

  printf("Connected.");
  app.Init(argc, argv);
  app.StartMainLoop();

  printf("Press Enter to exit program.\n");
  //getchar();

  DestroyConnection();

  return 0;
}
