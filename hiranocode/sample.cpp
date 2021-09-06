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
#include"LeapC.h"
#include <Springhead.h>
#include <Framework/SprFWApp.h>
//#include "FWFileLoaderSample.h"
#include "SampleApp.h"
#include "ExampleConnection.h"
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

    //Controller controller;
    int64_t lastFrameID = 0;
    double RealToSpr = 1 / 15;
    Vec3d Oya_position;
    Quaterniond Oya_orientation;
    PHSolidIf* soOyaBase;//Leapからの情報
    PHSolidIf* soOyaObj;//実際に触る方
    PHSpringIf* Oyajoint;
    PHSpringDesc Oyadesc;
    Posed oyapose;

    Vec3d Hito_position;
    Quaterniond Hito_orientation;
    PHSolidIf* soHitoBase;//Leapからの情報
    PHSolidIf* soHitoObj;//実際に触る方
    PHSpringIf* Hitojoint;
    PHSpringDesc Hitodesc;

    PHSpringDesc oyahitodesc;

    Quaterniond oyaori[3];
    Quaterniond hitoori[4];
    Quaterniond nakaori[4];
    Quaterniond kusuriori[4];
    Quaterniond koori[4];

    //Vec3d oya0;
    Vec3d oya1p;
    Vec3d oya2p;
    //Vec3d oya3p;
    //Vec3d hito0p;
    Vec3d hito1p;
    Vec3d hito2p;
    Vec3d hito3p;
    Vec3d naka0p;
    Vec3d naka1p;
    Vec3d naka2p;
    Vec3d naka3p;
    Vec3d kusuri0p;
    Vec3d kusuri1p;
    Vec3d kusuri2p;
    Vec3d kusuri3p;
    Vec3d ko0p;
    Vec3d ko1p;
    Vec3d ko2p;
    Vec3d ko3p;

    //Vec3d oya0n;
    Vec3d oya1n;
    Vec3d oya2n;
    Vec3d oya3n;
    //Vec3d hito0n;
    Vec3d hito1n;
    Vec3d hito2n;
    Vec3d hito3n;
    Vec3d naka0n;
    Vec3d naka1n;
    Vec3d naka2n;
    Vec3d naka3n;
    Vec3d kusuri0n;
    Vec3d kusuri1n;
    Vec3d kusuri2n;
    Vec3d kusuri3n;
    Vec3d ko0n;
    Vec3d ko1n;
    Vec3d ko2n;
    Vec3d ko3n;


    PHSolidIf* jenga1;
    PHSolidIf* jenga2;
    PHSolidIf* jenga3;
    PHSolidIf* jenga4;
    PHSolidIf* jenga5;
    PHSolidIf* jenga6;
    PHSolidIf* jenga7;
    PHSolidIf* jenga8;
    PHSolidIf* jenga9;




    //obj

    PHSolidIf* sooya1;
    PHSolidIf* sooya2;
    PHSolidIf* sohito1;
    PHSolidIf* sohito2;
    PHSolidIf* sohito3;
    PHSolidIf* sonaka0;
    PHSolidIf* sonaka1;
    PHSolidIf* sonaka2;
    PHSolidIf* sonaka3;
    PHSolidIf* sokusuri0;
    PHSolidIf* sokusuri1;
    PHSolidIf* sokusuri2;
    PHSolidIf* sokusuri3;
    PHSolidIf* soko0;
    PHSolidIf* soko1;
    PHSolidIf* soko2;
    PHSolidIf* soko3;



    //base

    PHSolidIf* sooya1b;
    PHSolidIf* sooya2b;
    PHSolidIf* sohito1b;
    PHSolidIf* sohito2b;
    PHSolidIf* sohito3b;
    PHSolidIf* sonaka0b;
    PHSolidIf* sonaka1b;
    PHSolidIf* sonaka2b;
    PHSolidIf* sonaka3b;
    PHSolidIf* sokusuri0b;
    PHSolidIf* sokusuri1b;
    PHSolidIf* sokusuri2b;
    PHSolidIf* sokusuri3b;
    PHSolidIf* soko0b;
    PHSolidIf* soko1b;
    PHSolidIf* soko2b;
    PHSolidIf* soko3b;

    PHSpringIf* oyajoint1;
    PHSpringIf* oyajoint2;
    PHSpringIf* hitojoint1;
    PHSpringIf* hitojoint2;
    PHSpringIf* hitojoint3;
    PHSpringIf* nakajoint0;
    PHSpringIf* nakajoint1;
    PHSpringIf* nakajoint2;
    PHSpringIf* nakajoint3;
    PHSpringIf* kusurijoint0;
    PHSpringIf* kusurijoint1;
    PHSpringIf* kusurijoint2;
    PHSpringIf* kusurijoint3;
    PHSpringIf* kojoint0;
    PHSpringIf* kojoint1;
    PHSpringIf* kojoint2;
    PHSpringIf* kojoint3;

    PHSpringIf* oyahito;

    PHSolidIf* judge;

    Quaterniond x90;
    Quaterniond y180;

    clock_t start, end;
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



    virtual void BuildScene() {

        soFloor = CreateFloor(true);

        //soPalm = CreateBone();




        //指先の作成

        //親指

        printf("手をかざしてお待ちください。\n");
        //Sleep(3);
        LEAP_TRACKING_EVENT* frame = GetFrame();
        LEAP_HAND* hand = &frame->pHands[0];
        //printf("hand is detected\n");
        float thunb_width;
        float index_width;
        float middle_width;
        float ring_width;
        float pinky_width;

        thunb_width = 20.0;
        index_width = 20.0;
        middle_width = 20.0;
        ring_width = 20.0;
        pinky_width = 20.0;




        thunb_width = hand->thumb.distal.width;
        index_width = hand->index.distal.width;
        middle_width = hand->middle.distal.width;
        ring_width = hand->ring.distal.width;
        pinky_width = hand->pinky.distal.width;

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



        thunb_width *= 1.2;
        index_width *= 1.2;

        soOyaBase = CreateBone2();
        soOyaBase->SetDynamical(false);
        GetFWScene()->SetSolidMaterial(GRRenderIf::RED, soOyaBase);
        printf("thumb_width=%f\n", thunb_width);
        //soOyaObj = CreateBone(thunb_width);
        soOyaObj = CreateBone(thunb_width);

        //soOyaObj->SetDynamical(true);
        GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, soOyaObj);
        //soOyaObj->SetMass(1);
        Oyadesc.spring = Vec3d(1500, 1500, 1500);
        Oyadesc.damper = Vec3d(15, 15, 15);
        Oyadesc.springOri = 10000.0;
        Oyadesc.damperOri = 1000.0;
        Oyajoint = GetPHScene()->CreateJoint(soOyaBase, soOyaObj, Oyadesc)->Cast();

        //人差し指
        soHitoBase = CreateBone2();
        soHitoBase->SetDynamical(false);
        GetFWScene()->SetSolidMaterial(GRRenderIf::RED, soHitoBase);
        //soHitoObj = CreateBone(index_width);
        soHitoObj = CreateBone(index_width);

        GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, soHitoObj);
        //soHitoObj->SetMass(1);
        Hitojoint = GetPHScene()->CreateJoint(soHitoBase, soHitoObj, Oyadesc)->Cast();


        //oyahitodesc.spring = Vec3d(200, 200, 200);
        //oyahitodesc.damper = Vec3d(15, 15, 15);
        //oyahitodesc.springOri = 100;
        //oyahitodesc.damperOri = 3;
        //oyahito= GetPHScene()->CreateJoint(soOyaObj, soHitoObj, oyahitodesc)->Cast();


        //
        ////指の作成

        ////obj


        sooya1 = CreateBone3(oya1n, oya1p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya1);
        sooya2 = CreateBone3(oya2n, oya2p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya2);


        sohito1 = CreateBone3(hito1n, hito1p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito1);
        sohito2 = CreateBone3(hito2n, hito2p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito2);
        sohito3 = CreateBone3(hito3n, hito3p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito3);

        sonaka0 = CreateBone5();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka0);
        sonaka1 = CreateBone3(naka1n, naka1p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka1);
        sonaka2 = CreateBone3(naka2n, naka2p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka2);
        sonaka3 = CreateBone3(naka3n, naka3p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka3);

        sokusuri0 = CreateBone5();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri0);
        sokusuri1 = CreateBone3(kusuri1n, kusuri1p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri1);
        sokusuri2 = CreateBone3(kusuri2n, kusuri2p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri2);
        sokusuri3 = CreateBone3(kusuri3n, kusuri3p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri3);

        soko0 = CreateBone5();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko0);
        soko1 = CreateBone3(ko1n, ko1p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko1);
        soko2 = CreateBone3(ko2n, ko2p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko2);
        soko3 = CreateBone3(ko3n, ko3p);
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko3);


        ////base


        sooya1b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya1b);
        sooya2b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sooya2b);

        sohito1b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito1b);
        sohito2b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito2b);
        sohito3b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sohito3);

        sonaka0b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka0b);
        sonaka1b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka1b);
        sonaka2b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka2b);
        sonaka3b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sonaka3b);

        sokusuri0b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri0b);
        sokusuri1b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri1b);
        sokusuri2b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri2b);
        sokusuri3b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, sokusuri3b);

        soko0b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko0b);
        soko1b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko1b);
        soko2b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko2b);
        soko3b = CreateBone2();
        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, soko3b);


        //joint
        oyajoint1 = GetPHScene()->CreateJoint(sooya1b, sooya1, Oyadesc)->Cast();
        oyajoint2 = GetPHScene()->CreateJoint(sooya2b, sooya2, Oyadesc)->Cast();
        hitojoint1 = GetPHScene()->CreateJoint(sohito1b, sohito1, Oyadesc)->Cast();
        hitojoint2 = GetPHScene()->CreateJoint(sohito2b, sohito2, Oyadesc)->Cast();
        hitojoint3 = GetPHScene()->CreateJoint(sohito3b, sohito3, Oyadesc)->Cast();
        nakajoint0 = GetPHScene()->CreateJoint(sonaka0b, sonaka0, Oyadesc)->Cast();
        nakajoint1 = GetPHScene()->CreateJoint(sonaka1b, sonaka1, Oyadesc)->Cast();
        nakajoint2 = GetPHScene()->CreateJoint(sonaka2b, sonaka2, Oyadesc)->Cast();
        nakajoint3 = GetPHScene()->CreateJoint(sonaka3b, sonaka3, Oyadesc)->Cast();
        kusurijoint0 = GetPHScene()->CreateJoint(sokusuri0b, sokusuri0, Oyadesc)->Cast();
        kusurijoint1 = GetPHScene()->CreateJoint(sokusuri1b, sokusuri1, Oyadesc)->Cast();
        kusurijoint2 = GetPHScene()->CreateJoint(sokusuri2b, sokusuri2, Oyadesc)->Cast();
        kusurijoint3 = GetPHScene()->CreateJoint(sokusuri3b, sokusuri3, Oyadesc)->Cast();
        kojoint0 = GetPHScene()->CreateJoint(soko0b, soko0, Oyadesc)->Cast();
        kojoint1 = GetPHScene()->CreateJoint(soko1b, soko1, Oyadesc)->Cast();
        kojoint2 = GetPHScene()->CreateJoint(soko2b, soko2, Oyadesc)->Cast();
        kojoint3 = GetPHScene()->CreateJoint(soko3b, soko3, Oyadesc)->Cast();



        //GetPHScene()->SetContactMode(soOyaBase,PHSceneDesc::MODE_NONE);

    }

    // タイマコールバック関数．タイマ周期で呼ばれる
    virtual void OnStep() {
        // GetSdk()->SaveScene("test.spr", NULL, FIFileSprIf::GetIfInfoStatic());

        SampleApp::OnStep();
        //printf("a");

        GetPHScene()->SetContactMode(sooya1b, PHSceneDesc::MODE_NONE);
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
        GetPHScene()->SetContactMode(soko0b, PHSceneDesc::MODE_NONE);



        GetPHScene()->SetContactMode(soOyaObj, soFloor, PHSceneDesc::MODE_NONE);
        GetPHScene()->SetContactMode(soHitoObj, soFloor, PHSceneDesc::MODE_NONE);
        GetPHScene()->SetContactMode(sonaka0, soFloor, PHSceneDesc::MODE_NONE);
        GetPHScene()->SetContactMode(sokusuri0, soFloor, PHSceneDesc::MODE_NONE);
        GetPHScene()->SetContactMode(soko0, soFloor, PHSceneDesc::MODE_NONE);

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
                LEAP_HAND* hand = &frame->pHands[h];

                Oya_position = vecvec3(hand->thumb.distal.prev_joint, hand->thumb.distal.next_joint);
                Oya_orientation = quaquad(hand->thumb.distal.rotation);
                Hito_position = vecvec3(hand->index.distal.prev_joint, hand->index.distal.next_joint);
                Hito_orientation = quaquad(hand->index.distal.rotation);
                //cout << Oya_position << endl;


                Oya_orientation = x90 * Oya_orientation;
                Oya_orientation = y180 * Oya_orientation;


                soOyaBase->SetFramePosition(Oya_position);
                soOyaBase->SetOrientation(Oya_orientation);
                //cout << soOyaBase->GetFramePosition() << endl;
                //cout << 'test' << endl;


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
                }
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
