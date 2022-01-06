/******************************************************************************\
* Copyright (C) 2012-2018 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <iostream>
#include <fstream>
//#include <Windows.h>
#include <cstring>
#include<chrono>
#include<thread>
//#include "LeapC++.h"
#include "../LeapSDK/include/LeapC.h"
#include <Springhead.h>
#include <Framework/SprFWApp.h>
#include "../src/Samples/SampleApp.h"
#include <string>
#include <sstream>
extern "C" {
#include "ExampleConnection.h"
}
#define USE_SPRFILE
#define ESC 27
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

    int type;
    int step_count;
    int leap_count;

    double scale = 2.0;
    double move_scale = 1.5;
    float friction = 100.0;
    float density = 1000.0;
    int contact = 0;
    int sorf = 0;//sucsess or failureの頭文字　0:タスク中　1:success 2:failure
    float cube_density = 1000.0;

    LEAP_TRACKING_EVENT* frame;
    LEAP_HAND* hand;

    //type 0
    PHSolidIf* fg_base[5][4];
    PHSolidIf* fg_obj[5][4];
    Vec3d fg_pos[5][4][2];
    Vec3d fg_pos_middle[5][4];
    Quaterniond fg_ori[5][4];
    PHSpringIf* fg_joint_vc[5][4];
    PHBallJointIf* fg_joint[5][3];

    PHHingeJointIf* fg_joint_hinge[5][3];
    PHHingeJointDesc fg_hinge_desc[5][3];

    //type 1
    //0はprevious、1はnext
    PHSolidIf* fg_base_slide[5][5][2];
    PHSolidIf* fg_obj_slide[5][4][2]; 
    PHSpringIf* fg_joint_vc_slide[5][4][2];
    PHSliderJointIf* fg_slider[5][4];
    PHSpringDesc spring_slide[5][4][2];
    PHBallJointDesc fg_ball_desc[5][3];
    PH1DJointLimitDesc slider_limit_desc;

    PHSpringIf* fg_palm_joint[3][2];

    PHBallJointIf* fg_joint_palm[3][2];
    PHBallJointDesc palm_ball_desc[3][2];
    

    PHSolidIf* palm_middle_base;
    PHSolidIf* palm_middle_obj;
    Quaterniond palm_middle_ori;
    PHSpringIf* palm_middle_joint_vc;

    Vec3d palm_middle_pos;
    PHSolidIf* palm_spread_obj[12];
    PHSolidIf* palm_spread_base[12];

    //Controller controller;
    int64_t lastFrameID = 0;
    double RealToSpr = 1 / 15;

    PHSpringDesc Springdesc;
    PHBallJointDesc Balldesc[5][3];
    PHSliderJointDesc Sliderdesc;

    PHSolidIf* cube;//落とす物体

    PHSpringDesc oyahitodesc;

    PHSolidIf* cylinder;
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

        //GetPHScene()->SetNumIteration(75);

        

        //何番目のシーンなのかを取得して、そのシーンについてfindobjectをする
        /*int n;
        n=GetSdk()->NScene();
        DSTR << n << endl;*/
        //string filename;
        //filename = "./file/scene.spr";
        //UTRef<ImportIf> import = GetSdk()->GetFISdk()->CreateImport();
        //GetSdk()->LoadScene(filename, import);
        //UTString objectname;
        //objectname = "soCylinder";
        //cylinder = GetPHScene()->FindObject(objectname)->Cast();

        //n = GetSdk()->NScene();
        //DSTR << n << endl;

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

        // 0:no slider
        // 1:slider
        // 2:delete sliderjoint
        // 3:0の改良。指先にしか当たり判定が無い
        // 4:オートで操作
        //type = 2;

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



    PHSolidIf* CreateFloor(bool bWall) {
        PHSolidIf* soFloor = GetPHScene()->CreateSolid();
        soFloor->SetName("soFloor");
        soFloor->SetDynamical(false);
        double from_center;
        double width;
        from_center = 0.5;
        width = 0.25;
        soFloor->AddShape(shapeFloor);
        soFloor->SetShapePose(0, Posed::Trn(30.0+from_center+width, -shapeFloor->GetBoxSize().y /2+0.1, 0));
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
        soFloor->SetShapePose(5, Posed::Trn(-30.0+from_center, -shapeFloor->GetBoxSize().y / 2 + 0.1, 0));
        soFloor->SetShapePose(6, Posed::Trn(0, -shapeFloor->GetBoxSize().y / 2 + 0.1, 20.0+width/2));
        soFloor->SetShapePose(7, Posed::Trn(0, -shapeFloor->GetBoxSize().y / 2 + 0.1, -20.0 - width / 2));
        CDBoxDesc wz;
        wz = Vec3d(0.1, 0.7, 40.0);
        shapeWallZ = GetSdk()->GetPHSdk()->CreateShape(wz)->Cast();
        soFloor->AddShape(shapeWallZ);
        soFloor->SetShapePose(8, Posed::Trn(0.3, 0, 0));
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
        nail.radius = Vec3d(1.69f, 0.179f, 1.12f) * (double)BlenderToSpr;
        shapenail = GetSdk()->GetPHSdk()->CreateShape(nail)->Cast();
        shapenail->SetDensity(density);
        shapenail->SetStaticFriction(friction);
        shapenail->SetDynamicFriction(friction);

        CDEllipsoidDesc pad;
        pad.radius = Vec3d(2.79, 1.26, 1.5) * (double)BlenderToSpr;
        shapepad = GetSdk()->GetPHSdk()->CreateShape(pad)->Cast();
        shapepad->SetDensity(density);
        shapepad->SetStaticFriction(friction);
        shapepad->SetDynamicFriction(friction);

        CDCapsuleDesc bone;
        bone.radius = 1.24 * BlenderToSpr;
        bone.length = (2.0f - 1.24f) * 2.0f * BlenderToSpr;
        shapebone = GetSdk()->GetPHSdk()->CreateShape(bone)->Cast();
        shapebone->SetDensity(density);
        shapebone->SetStaticFriction(friction);
        shapebone->SetDynamicFriction(friction);

        soBone->AddShape(shapenail);
        soBone->AddShape(shapepad);
        soBone->AddShape(shapebone);


        soBone->SetShapePose(0, Posed(Vec3d(0.0, 0.31 * 2.0, -0.67 * 2.0) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90.0), 'y')));
        soBone->SetShapePose(1, Posed(Vec3d(0.0, 0.0, 0.0) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90.0), 'y')));
        soBone->SetShapePose(2, Posed(Vec3d(0.0, 0.0, 0.38 * 2.0) * (double)BlenderToSpr, Quaterniond::Rot(Rad(0.0), 'y')));//180かも

        //soBone->CompInertia();
        soBone->SetGravity(false);
        return soBone;

    }

    //指先(爪なし)
    PHSolidIf* CreateBoneNo(float width) {

        float BlenderToSpr = width / 2.48 / 15.0;
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
        shapepad->SetDensity(density);
        shapepad->SetStaticFriction(friction);
        shapepad->SetDynamicFriction(friction);

        CDCapsuleDesc bone;
        bone.radius = 1.24f * BlenderToSpr;
        bone.length = (2.0f - 1.24f) * 2.0f * BlenderToSpr;
        shapebone = GetSdk()->GetPHSdk()->CreateShape(bone)->Cast();
        shapebone->SetDensity(density);
        shapebone->SetStaticFriction(friction);
        shapebone->SetDynamicFriction(friction);

        //soBone->AddShape(shapenail);
        soBone->AddShape(shapepad);
        soBone->AddShape(shapebone);


        //soBone->SetShapePose(0, Posed(Vec3d(0, 0.31 * 2, -0.67 * 2) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90), 'y')));
        soBone->SetShapePose(0, Posed(Vec3d(0.0, 0.0, 0.0) * (double)BlenderToSpr, Quaterniond::Rot(Rad(-90.0), 'y')));
        soBone->SetShapePose(1, Posed(Vec3d(0.0, 0.0, 0.38 * 2) * (double)BlenderToSpr, Quaterniond::Rot(Rad(0.0), 'y')));//180かも

        soBone->CompInertia();
        //soBone->SetMass(1.0f);
        //soBone->SetGravity(false);
        return soBone;

    }

    //fg_base用のsolid
    PHSolidIf* CreateBone_base() {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDSphereDesc sd;
        sd.radius = 1.0f / 1000.0f * (float)scale;

        shapeSphere = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();

        soBone->SetDynamical(false);


        soBone->AddShape(shapeSphere);


        return soBone;
    }
    //fg_obj用のsolid(カプセル)
    PHSolidIf* CreateBoneCapsule(Vec3d p, Vec3d n) {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDCapsuleDesc cd;

        //8.5
        cd.radius = 8.5f/1000.0f * (float)scale;
        cd.length = (float)sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y) + (p.z - n.z) * (p.z - n.z));


        shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
        shapeCapsule->SetDensity(density);
        shapeCapsule->SetStaticFriction(friction);
        shapeCapsule->SetDynamicFriction(friction);

        soBone->AddShape(shapeCapsule);

        soBone->SetGravity(false);

        //soBone->CompInertia();

        return soBone;
    }
    //fg_obj用のsolid(カプセル)を回転
    PHSolidIf* CreateBoneCapsuleRound(Vec3d p, Vec3d n, Quaterniond q) {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDCapsuleDesc cd;

        //8.5
        cd.radius = 8.5f / 1000.0f * (float)scale;
        cd.length = (float)sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y) + (p.z - n.z) * (p.z - n.z));
        cd.length /= 2.0;

        shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
        shapeCapsule->SetDensity(density);
        shapeCapsule->SetStaticFriction(friction);
        shapeCapsule->SetDynamicFriction(friction);

        soBone->AddShape(shapeCapsule);
        
        soBone->SetShapePose(0, Posed(Vec3d(), q));
        soBone->SetGravity(false);

        soBone->CompInertia();

        return soBone;
    }
    //fg_obj用のsolid(roundcone)
    PHSolidIf* CreateBoneRoundCone(Vec3d p, Vec3d n) {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDRoundConeDesc rd;

        //8.5
        rd.radius = Vec2d(1.0,1.0)*8.5f / 1000.0f * (float)scale;
        rd.length = (float)sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y) + (p.z - n.z) * (p.z - n.z));


        shapeRoundCone = GetSdk()->GetPHSdk()->CreateShape(rd)->Cast();
        shapeRoundCone->SetDensity(density);
        shapeRoundCone->SetStaticFriction(friction);
        shapeRoundCone->SetDynamicFriction(friction);

        soBone->AddShape(shapeRoundCone);

        soBone->SetGravity(false);

        soBone->CompInertia();

        return soBone;
    }


    PHSolidIf* CreateBoneSlide(Vec3d p, Vec3d n) {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDCapsuleDesc cd;

        //1.0->2.0に修正
        cd.radius = 8.5f/1000.0f * (float)scale;
        cd.length = (float)sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y) + (p.z - n.z) * (p.z - n.z));
        cd.length /= 1.5;

        cout << "radius="<<cd.radius << endl;
        cout << "length=" << cd.length << endl;

        shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
        shapeCapsule->SetDensity(density);
        shapeCapsule->SetStaticFriction(friction);
        shapeCapsule->SetDynamicFriction(friction);

        soBone->AddShape(shapeCapsule);

        soBone->SetGravity(false);

        soBone->CompInertia();

        return soBone;
    }
    //cube
    PHSolidIf* CreateBoneCube() {
        
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDBoxIf* shapeBone;
        CDBoxDesc sd;
        sd.boxsize = Vec3d(3.0, 3.0, 3.0)/100.0;

        shapeBone = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();
        shapeBone->SetDensity(density);
        shapeBone->SetStaticFriction(friction);
        shapeBone->SetDynamicFriction(friction);

        //soBone->SetDynamical(true);
        //soBone->SetName("soBone");

        soBone->AddShape(shapeBone);

        soBone->SetGravity(false);
        soBone->CompInertia();
        return soBone;
    }
    //sphere
    PHSolidIf* CreateBoneSphere() {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDSphereIf* Bone5;
        CDSphereDesc sd;
        sd.radius = 10.0/1000.0*scale;

        Bone5 = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();
        Bone5->SetStaticFriction(friction);
        Bone5->SetDynamicFriction(friction);
        Bone5->SetDensity(density);

        soBone->AddShape(Bone5);
        soBone->SetGravity(false);
        //soBone->CompInertia();

        return soBone;
    }
    PHSolidIf* CreateBoneSphereShift() {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDSphereIf* Bone5;
        CDSphereDesc sd;
        sd.radius = 1.0 / 100.0*scale;

        Bone5 = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();
        Bone5->SetStaticFriction(friction);
        Bone5->SetDynamicFriction(friction);
        Bone5->SetDensity(density);

        soBone->AddShape(Bone5);

        //soBone->CompInertia();
        Vec3d shift;
        shift = Vec3d(0.0, 0.0, -0.02);
        soBone->SetCenterOfMass(shift);


        return soBone;
    }
    PHSolidIf* CreatePalm_obj() {
        PHSolidIf* soPalm = GetPHScene()->CreateSolid();
        CDBoxIf* shapepalm;
        CDBoxDesc palm;
        palm.boxsize = Vec3d(3.5, 1.0, 4.0)/1000.0*scale;
        shapepalm = GetSdk()->GetPHSdk()->CreateShape(palm)->Cast();
        soPalm->AddShape(shapepalm);
        soPalm->SetShapePose(0, Posed(Vec3d(0.0, 0.0, 2.5)/1000.0*scale, Quaterniond::Rot(Rad(0.0), 'y')));
        soPalm->SetGravity(false);
        return soPalm;
    }



    Vec3d vecvec3(LEAP_VECTOR vec, LEAP_VECTOR vec2) {
        Vec3d vec3;
        double x;
        double y;
        double z;
        //x = (double)((double)vec.x + (double)vec2.x);
        //y = (double)((double)vec.y + (double)vec2.y);
        //z = (double)((double)vec.z + (double)vec2.z);
        //x /= 2;
        //y /= 2;
        //z /= 2;

        //vec3.x = -x / 10.0;
        //vec3.y = 25.0 - z / 10.0;
        //vec3.z = 30.0 - y / 10.0;

        x = (double)((double)vec.x + (double)vec2.x)/1000.0;
        y = (double)((double)vec.y + (double)vec2.y)/1000.0;
        z = (double)((double)vec.z + (double)vec2.z)/1000.0;
        x /= 2;
        y /= 2;
        z /= 2;

        vec3.x = -x * scale;
        vec3.y = (0.50 - z) * scale;
        vec3.z = (0.30 - y) * scale;


        return vec3;
    }
    Vec3d vecvec3_2(LEAP_VECTOR vec) {
        Vec3d vec3;
        //vec3.x = (double)(vec.x / 10.0);
        //vec3.y = (double)(vec.y / 10.0);
        //vec3.z = (double)(vec.z / 10.0);

        vec3.x = (double)vec.x / 1000.0 * scale;
        vec3.y = (double)vec.y / 1000.0 * scale;
        vec3.z = (double)vec.z / 1000.0 * scale;

        return vec3;
    }
    Vec3d vecvec3_3(LEAP_VECTOR vec) {
        Vec3d vec3;
        double x;
        double y;
        double z;
        //x = (double)(vec.x / 10.0);
        //y = (double)(vec.y / 10.0);
        //z = (double)(vec.z / 10.0);

        //vec3.x = -x;
        //vec3.y = 30.0 - z;
        //vec3.z = 30.0 - y;

        x = (double)vec.x / 1000.0;
        y = (double)vec.y / 1000.0;
        z = (double)vec.z / 1000.0;

        vec3.x = -x * scale;
        vec3.y = (0.50 - z) * scale;
        vec3.z = (0.30 - y) * scale;

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
    double jointLength(Vec3d vec1, Vec3d vec2) {
        double length;
        length = (vec1.x - vec2.x) * (vec1.x - vec2.x) + (vec1.y - vec2.y) * (vec1.y - vec2.y) + (vec1.z - vec2.z) * (vec1.z - vec2.z);
        length = sqrt(length);
        return length;
    }

    void Drop(int shape, int mat, Vec3d v, Vec3d w, Vec3d p, Quaterniond q) {
        // ステートを解放
        states->ReleaseState(GetPHScene());

        // 剛体を作成
        //solid= GetPHScene()->CreateSolid();
        /*PHSolidIf* solid = GetPHScene()->CreateSolid();*/
        // マテリアルを設定
        GetFWScene()->SetSolidMaterial(mat, cube);
        
        // 形状の割当て
        if (shape == SHAPE_BOX) {
            CDBoxDesc bd;
            bd.boxsize = Vec3d(4.0, 4.0, 4.0)/100.0 * scale;  //単位 m^3
            shapeBox->SetDynamicFriction(friction);
            shapeBox->SetStaticFriction(friction);
            shapeBox->SetDensity(cube_density);//単位 kg/m^3
            shapeBox = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
            cube->AddShape(shapeBox);
        }

        if (shape == SHAPE_CAPSULE) {
            CDCapsuleDesc cd;
            cd.radius=1.0f / 100.0f * (float)scale;
            cd.length = 10.0f / 100.0f * (float)scale;
            shapeCapsule->SetDynamicFriction(friction);
            shapeCapsule->SetStaticFriction(friction);
            shapeCapsule->SetDensity(cube_density);
            shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
            cube->AddShape(shapeCapsule);

        }

        if (shape == SHAPE_ROUNDCONE)
            cube->AddShape(shapeRoundCone);
        if (shape == SHAPE_SPHERE)
            cube->AddShape(shapeSphere);
        if (shape == SHAPE_ELLIPSOID){
            //solid->AddShape(shapeEllipsoid);
            CDBoxDesc bd;
            bd.boxsize = Vec3d(3.0, 15.0, 3.0) / 100.0 * scale;  //単位 m^3
            shapeBox->SetDynamicFriction(friction);
            shapeBox->SetStaticFriction(friction);
            shapeBox->SetDensity(cube_density);//単位 kg/m^3
            shapeBox = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
            cube->AddShape(shapeBox);

        }
            
        if (shape == SHAPE_ROCK) {
            CDConvexMeshDesc md;
            int nv = rand() % 100 + 50;
            for (int i = 0; i < nv; ++i) {
                Vec3d v;
                for (int c = 0; c < 3; ++c) {
                    v[c] = ((rand() % 100) / 100.0 - 0.5) * 5 * 1.3 * ShapeScale();
                }
                md.vertices.push_back(v);
            }
            cube->AddShape(GetSdk()->GetPHSdk()->CreateShape(md));
        }
        if (shape == SHAPE_BLOCK) {
            for (int i = 0; i < 7; i++)
                cube->AddShape(shapeBox);
            Posed pose;
            pose.Pos() = ShapeScale() * Vec3d(3, 0, 0); cube->SetShapePose(1, pose);
            pose.Pos() = ShapeScale() * Vec3d(-3, 0, 0); cube->SetShapePose(2, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, 3, 0); cube->SetShapePose(3, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, -3, 0); cube->SetShapePose(4, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, 0, 3); cube->SetShapePose(5, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, 0, -3); cube->SetShapePose(6, pose);
        }
        if (shape == SHAPE_COIN) {
            cube->AddShape(shapeCoin);
        }


        if (type == 1||type==2||type==3) {

            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {

                        GetPHScene()->SetContactMode(fg_base_slide[i][j][0], cube, PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_base_slide[i][j][1], cube, PHSceneDesc::MODE_NONE);

                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][0], cube, PHSceneDesc::MODE_LCP);
                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][1], cube, PHSceneDesc::MODE_LCP);
                        if (j == 3) {
                            GetPHScene()->SetContactMode(fg_obj[i][j], cube, PHSceneDesc::MODE_LCP);
                        }

                    }
                }
            }
            
        }
        cube->SetVelocity(v);
        cube->SetAngularVelocity(w);
        p = Vec3d(0.0, 0.25, 0.0);
        cube->SetFramePosition(p);
        cube->SetOrientation(q);
        cube->CompInertia();
    }
    virtual void BuildScene() {

        PHSceneDesc pd;
        GetPHScene()->GetDesc(&pd);
        pd.timeStep = 1.0 / 60;
        pd.contactTolerance = 0.001 * 0.4;
        pd.airResistanceRateForAngularVelocity = 0.98;
        GetPHScene()->SetDesc(&pd);
        PHConstraintEngineDesc ed;
        GetPHScene()->GetConstraintEngine()->GetDesc(&ed);
        ed.freezeThreshold = 0;
        ed.contactCorrectionRate = 0.5;
        GetPHScene()->GetConstraintEngine()->SetDesc(&ed);
        GetPHScene()->SetGravity(Vec3f(0.0f, -9.8f*0.5f, 0.0f));	// 重力を設定
        GetPHScene()->SetTimeStep(0.015);
        GetPHScene()->SetNumIteration(75);

        soFloor = CreateFloor(true);
        GetCurrentWin()->GetTrackball()->SetPosition(Vec3d(0.0,2.5,10.0));

        GetFWScene()->EnableRenderAxis(false, false, false);
        //GetFWScene()->EnableRenderForce(false, false);
        GetFWScene()->SetForceScale(0.001f, 0.001f);
        printf("手をかざしてお待ちください。\n");
        cout << "0:no slider\n1:slider\n2:delete sliderjoint\n3:contact on fingertip only\n4:auto\n5:two finger" << endl;
        cout << "type = ";
        cin >> type;

        int detect;
        LEAP_TRACKING_EVENT* frame;
        if (type != 4) {
            detect = 0;
            while (detect == 0) {
                frame = GetFrame();
                //printf("%d\n", (int)frame->nHands);
                if ((int)frame->nHands > 0) {
                    detect = 1;
                }
            }
            hand = &frame->pHands[0];
        }
        cube = GetPHScene()->CreateSolid();
        //LEAP_TRACKING_EVENT* frame = GetFrame();
        
        //printf("hand is detected\n");

        float tip_width[5] = { 0.0 };



        double spring;
        double damper;
        spring = 10000.0;
        damper = 10.0;
        Springdesc.spring = 0.5*Vec3d(spring, spring, spring);
        Springdesc.damper = 0.5*Vec3d(damper, damper, damper);
        Springdesc.springOri = spring;
        Springdesc.damperOri = damper;

        if (type == 0) {

            for (int i = 0; i < 5; i++) {
                tip_width[i] = hand->digits[i].distal.width;
                //tip_width[i] *= 1.5;
            }
            //
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_pos[i][j][0] = vecvec3_2(hand->digits[i].bones[j].prev_joint);
                        fg_pos[i][j][1] = vecvec3_2(hand->digits[i].bones[j].next_joint);
                    }
                }
            }
            //
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {

                        fg_base[i][j] = CreateBone_base();
                        GetFWScene()->SetSolidMaterial(GRRenderIf::RED, fg_base[i][j]);
                        if (j == 3) {//指先
                            fg_obj[i][j] = CreateBone(tip_width[i]);
                        }
                        else {//指の骨
                            fg_obj[i][j] = CreateBoneCapsule(fg_pos[i][j][0], fg_pos[i][j][1]);
                        }

                        GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj[i][j]);

                        fg_joint_vc[i][j] = GetPHScene()->CreateJoint(fg_base[i][j], fg_obj[i][j], Springdesc)->Cast();
                    }
                }
            }
            //
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 3; j++) {
                    if (!(i == 0 && j == 0)) {
                        Balldesc[i][j].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1])) / 2.0;
                        Balldesc[i][j].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j + 1][0], fg_pos[i][j + 1][1])) / 2.0;
                        fg_joint[i][j] = GetPHScene()->CreateJoint(fg_obj[i][j], fg_obj[i][j + 1], Balldesc[i][j])->Cast();
                    }
                }
            }
            //
            GetPHScene()->SetContactMode(&(fg_obj[0][1]), 4 * 5 - 1, PHSceneDesc::MODE_NONE);
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        GetPHScene()->SetContactMode(fg_base[i][j], PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_obj[i][j], soFloor, PHSceneDesc::MODE_NONE);
                    }
                }
            }
        }
        else if (type == 1) {

            for (int i = 0; i < 5; i++) {
                tip_width[i] = hand->digits[i].distal.width;
                //tip_width[i] *= 1.5;
            }

            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_pos[i][j][0] = vecvec3_2(hand->digits[i].bones[j].prev_joint);
                        fg_pos[i][j][1] = vecvec3_2(hand->digits[i].bones[j].next_joint);
                    }
                }
            }
            //インターフェース側のsolid作成
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_base_slide[i][j][0] = CreateBone_base();
                        fg_base_slide[i][j][1] = CreateBone_base();
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][0]);
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][1]);

                        GetPHScene()->SetContactMode(fg_base_slide[i][j][0], PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_base_slide[i][j][1], PHSceneDesc::MODE_NONE);

                    }
                }
            }


            //ツール側のsolid作成
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        if (j == 3) {
                            fg_obj_slide[i][j][0]= CreateBoneSlide(fg_pos[i][j][0], fg_pos[i][j][1]);
                            fg_obj_slide[i][j][1] = CreateBoneNo(tip_width[i]*0.013*scale);
                            }
                        else
                        {
                            fg_obj_slide[i][j][0] = CreateBoneSlide(fg_pos[i][j][0], fg_pos[i][j][1]);
                            fg_obj_slide[i][j][1] = CreateBoneSlide(fg_pos[i][j][0], fg_pos[i][j][1]);
                        }
                        if (j == 0) {
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][1]);
                        }
                        else if(j==1){
                            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj_slide[i][j][1]);
                        }
                        else if (j == 2) {
                            GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_obj_slide[i][j][1]);
                        }
                        else if(j==3){
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][1]);
                        }



                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][0], PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][1], PHSceneDesc::MODE_NONE);

                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][0],soFloor ,PHSceneDesc::MODE_LCP);
                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][1],soFloor, PHSceneDesc::MODE_LCP);

                    }
                }
            }
            //GetPHScene()->SetContactMode(&(fg_obj_slide[0][1][0]), 5*4*2-2, PHSceneDesc::MODE_NONE);

            //バーチャルカップリング
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {

                        spring_slide[i][j][0].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j][0], fg_pos[i][j][1])/4.0);
                        spring_slide[i][j][1].posePlug.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1])/4.0);
                        
                        spring_slide[i][j][0].spring = Vec3d(1000.0, 1000.0, 1000.0);
                        spring_slide[i][j][0].damper = Vec3d(10.0, 10.0, 10.0);
                        spring_slide[i][j][0].springOri = 1000.0;
                        spring_slide[i][j][0].damperOri = 10.0;

                        spring_slide[i][j][1].spring = Vec3d(1000.0, 1000.0, 1000.0);
                        spring_slide[i][j][1].damper = Vec3d(10.0, 10.0, 10.0);
                        spring_slide[i][j][1].springOri = 1000.0;
                        spring_slide[i][j][1].damperOri = 10.0;


                        fg_joint_vc_slide[i][j][0] = GetPHScene()->CreateJoint(fg_base_slide[i][j][0], fg_obj_slide[i][j][0], spring_slide[i][j][0])->Cast();
                        fg_joint_vc_slide[i][j][1] = GetPHScene()->CreateJoint(fg_base_slide[i][j][1], fg_obj_slide[i][j][1], spring_slide[i][j][1])->Cast();
                    }
                }
            }
            slider_limit_desc.range = Vec2d(0.0, 0.0);
            //slider_limit_desc.
            slider_limit_desc.spring = 1000000000000.0;
            slider_limit_desc.damper = 1000000000000.0;
            //スライダージョイント
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        Sliderdesc.spring = 1000.0;
                        Sliderdesc.damper = 10.0;
                        fg_slider[i][j] = GetPHScene()->CreateJoint(fg_obj_slide[i][j][0], fg_obj_slide[i][j][1], Sliderdesc)->Cast();
                        fg_slider[i][j]->CreateLimit(slider_limit_desc);
                        //fg_slider[i][j]->SetTargetPosition()
                    }
                }
            }
            //ボールジョイント
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 3;j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_ball_desc[i][j].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 4.0);
                        fg_ball_desc[i][j].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j+1][0], fg_pos[i][j+1][1]) / 4.0);
                        fg_joint[i][j] = GetPHScene()->CreateJoint(fg_obj_slide[i][j][1], fg_obj_slide[i][j + 1][0], fg_ball_desc[i][j])->Cast();
                    }
                }
            }

            for (int i = 0; i < 3; i++) {
                palm_ball_desc[i][0].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i+1][0][0], fg_pos[i+1][0][1]) / 4.0);
                palm_ball_desc[i][0].posePlug.Pos() = Vec3d(-8.5f / 1000.0f * (float)scale*2, 0.0, -jointLength(fg_pos[i + 2][0][0], fg_pos[i + 2][0][1]) / 4.0);// + fg_pos[i + 1][0][1] - fg_pos[i + 2][0][1]
                fg_joint_palm[i][0] = GetPHScene()->CreateJoint(fg_obj_slide[i+1][0][1], fg_obj_slide[i+2][0][1], palm_ball_desc[i][0])->Cast();
                
            }
            //形の値の変更
            //CDCapsuleIf* cp= fg_obj_slide[1][1][1]->GetShape(0)->Cast();
            //cp->SetLength(1.0f);
            //fg_obj_slide[1][1][1]->InvalidateBbox();
        }
        else if (type == 2) {
            for (int i = 0; i < 5; i++) {
                tip_width[i] = hand->digits[i].distal.width;
                //tip_width[i] *= 1.5;
            }
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_pos[i][j][0] = vecvec3_2(hand->digits[i].bones[j].prev_joint);
                        fg_pos[i][j][1] = vecvec3_2(hand->digits[i].bones[j].next_joint);
                    }
                }
            }
            //インターフェース側のsolid作成
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_base_slide[i][j][0] = CreateBone_base();
                        fg_base_slide[i][j][1] = CreateBone_base();
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][0]);
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][1]);

                        GetPHScene()->SetContactMode(fg_base_slide[i][j][0], PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_base_slide[i][j][1], PHSceneDesc::MODE_NONE);

                    }
                }
            }


            //ツール側のsolid作成
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        if (j == 3) {
                            fg_obj_slide[i][j][0] = CreateBoneSlide(fg_pos[i][j][0], fg_pos[i][j][1]);
                            fg_obj_slide[i][j][1] = CreateBoneNo(tip_width[i] * 0.013 * scale);
                        }
                        else
                        {
                            fg_obj_slide[i][j][0] = CreateBoneSlide(fg_pos[i][j][0], fg_pos[i][j][1]);
                            fg_obj_slide[i][j][1] = CreateBoneSlide(fg_pos[i][j][0], fg_pos[i][j][1]);
                        }
                        if (j == 0) {
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][1]);
                        }
                        else if (j == 1) {
                            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj_slide[i][j][1]);
                        }
                        else if (j == 2) {
                            GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_obj_slide[i][j][1]);
                        }
                        else if (j == 3) {
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][0]);
                            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][1]);
                        }



                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][0], PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][1], PHSceneDesc::MODE_NONE);

                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][0], soFloor, PHSceneDesc::MODE_LCP);
                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][1], soFloor, PHSceneDesc::MODE_LCP);

                    }
                }
            }
            //GetPHScene()->SetContactMode(&(fg_obj_slide[0][1][0]), 5*4*2-2, PHSceneDesc::MODE_NONE);

            //バーチャルカップリング
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {

                        spring_slide[i][j][0].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 4.0);
                        spring_slide[i][j][1].posePlug.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 4.0);

                        spring_slide[i][j][0].spring = Vec3d(1000.0, 1000.0, 1000.0);
                        spring_slide[i][j][0].damper = Vec3d(10.0, 10.0, 10.0);
                        spring_slide[i][j][0].springOri = 1000.0;
                        spring_slide[i][j][0].damperOri = 10.0;

                        spring_slide[i][j][1].spring = Vec3d(1000.0, 1000.0, 1000.0);
                        spring_slide[i][j][1].damper = Vec3d(10.0, 10.0, 10.0);
                        spring_slide[i][j][1].springOri = 1000.0;
                        spring_slide[i][j][1].damperOri = 10.0;


                        fg_joint_vc_slide[i][j][0] = GetPHScene()->CreateJoint(fg_base_slide[i][j][0], fg_obj_slide[i][j][0], spring_slide[i][j][0])->Cast();
                        fg_joint_vc_slide[i][j][1] = GetPHScene()->CreateJoint(fg_base_slide[i][j][1], fg_obj_slide[i][j][1], spring_slide[i][j][1])->Cast();
                    }
                }
            }

            //ボールジョイント
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 3; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_ball_desc[i][j].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 4.0);
                        fg_ball_desc[i][j].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j + 1][0], fg_pos[i][j + 1][1]) / 4.0);
                        fg_joint[i][j] = GetPHScene()->CreateJoint(fg_obj_slide[i][j][1], fg_obj_slide[i][j + 1][0], fg_ball_desc[i][j])->Cast();
                    }
                }
            }
            //手のひらの拘束
            //for (int i = 0; i < 3; i++) {
            //    palm_ball_desc[i][0].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i + 1][0][0], fg_pos[i + 1][0][1]) / 4.0);
            //    palm_ball_desc[i][0].posePlug.Pos() = Vec3d(-8.5f / 1000.0f * (float)scale * 2, 0.0, -jointLength(fg_pos[i + 2][0][0], fg_pos[i + 2][0][1]) / 4.0);// + fg_pos[i + 1][0][1] - fg_pos[i + 2][0][1]
            //    fg_joint_palm[i][0] = GetPHScene()->CreateJoint(fg_obj_slide[i + 1][0][1], fg_obj_slide[i + 2][0][1], palm_ball_desc[i][0])->Cast();

            //}
        }
        else if (type == 3) {
            //位置取得
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_pos[i][j][0] = vecvec3_2(hand->digits[i].bones[j].prev_joint);
                        fg_pos[i][j][1] = vecvec3_2(hand->digits[i].bones[j].next_joint);
                    }
                }
            }
            //base
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_base_slide[i][j][0] = CreateBone_base();
                        fg_base_slide[i][j][1] = CreateBone_base();
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][0]);
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][1]);
                    }
                }
            }
            //tool
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        if (j == 3) {
                            //fg_obj[i][j] = CreateBoneCapsule(fg_pos[i][j][0], fg_pos[i][j][1]);
                            //fg_obj[i][j] = CreateBoneSphere();
                            fg_obj[i][j] = CreateBone(0.45);
                        }
                        else {
                            fg_obj[i][j] = CreateBoneCapsule(fg_pos[i][j][0], fg_pos[i][j][1]);

                        }
                        GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE,fg_obj[i][j]);

                        GetPHScene()->SetContactMode(fg_obj[i][j], PHSceneDesc::MODE_NONE);
                    }
                }
            }
            //バーチャルカップリング
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        spring_slide[i][j][0].posePlug.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 2.0);
                        spring_slide[i][j][1].posePlug.Pos() = Vec3d(0.0, 0.0,  jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 2.0);
                        spring = 10000.0;
                        damper = 10.0;
                        spring_slide[i][j][0].spring = 0.1*Vec3d(spring, spring, spring);
                        spring_slide[i][j][0].damper = 0.1*Vec3d(damper, damper, damper);
                        spring_slide[i][j][0].springOri = spring;
                        spring_slide[i][j][0].damperOri = damper;

                        spring_slide[i][j][1].spring = 0.1*Vec3d(spring, spring, spring);
                        spring_slide[i][j][1].damper = 0.1*Vec3d(damper, damper, damper);
                        spring_slide[i][j][1].springOri = spring;
                        spring_slide[i][j][1].damperOri = damper;
                        
                        fg_joint_vc_slide[i][j][0] = GetPHScene()->CreateJoint(fg_base_slide[i][j][0], fg_obj[i][j], spring_slide[i][j][0])->Cast();
                        fg_joint_vc_slide[i][j][1] = GetPHScene()->CreateJoint(fg_base_slide[i][j][1], fg_obj[i][j], spring_slide[i][j][1])->Cast();
                    }
                }
            }
            //balljoint
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 3; j++) {
                    if (!(i == 0 && j == 0)) {
                        fg_ball_desc[i][j].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 2.0);
                        fg_ball_desc[i][j].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j + 1][0], fg_pos[i][j + 1][1]) / 2.0);
                        fg_joint[i][j] = GetPHScene()->CreateJoint(fg_obj[i][j], fg_obj[i][j + 1], fg_ball_desc[i][j])->Cast();
                    }
                }
            }
        }
        if (type == 4) {
            //cubeの設置
            Vec3d v, w(0.0, 0.0, 0.2), p(0.5, 10, 0.0);
            static Quaterniond q = Quaterniond::Rot(Rad(0.0), 'y');
            q = Quaterniond::Rot(Rad(90), 'y') * q;
            CDBoxDesc bd;
            bd.boxsize = Vec3d(4.0, 8.0, 4.0) / 100.0 * scale;  //単位 m^3
            shapeBox->SetDynamicFriction(friction);
            shapeBox->SetStaticFriction(friction);
            shapeBox->SetDensity(cube_density);//単位 kg/m^3
            shapeBox = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
            cube->AddShape(shapeBox);
            cube->SetVelocity(v);
            cube->SetAngularVelocity(w);
            p = Vec3d(0.0, 0.25, 0.0);
            cube->SetFramePosition(p);
            cube->SetOrientation(q);
            cube->CompInertia();


            //インターフェース側の剛体の作成
            fg_base[0][3] = CreateBone_base();
            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_base[0][3]);
            fg_base[1][3] = CreateBone_base();
            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_base[1][3]);

            GetPHScene()->SetContactMode(fg_base[0][3], PHSceneDesc::MODE_NONE);
            GetPHScene()->SetContactMode(fg_base[1][3], PHSceneDesc::MODE_NONE);


            //tool側の剛体の作成
            // 
            // sphereは球、capsuleは長さ0.2の長さのカプセル、capsuleRoundは0.2の長さのカプセルを回転させられる
            // 
            //fg_obj[0][3] = CreateBoneSphere();
            //fg_obj[0][3] = CreateBoneCapsule(Vec3d(0.2, 0.0, 0.0), Vec3d(0.0, 0.0, 0.0));
            fg_obj[0][3] = CreateBoneCapsuleRound(Vec3d(0.2, 0.0, 0.0), Vec3d(0.0, 0.0, 0.0), Quaterniond::Rot(Rad(-30.0), 'x'));
           
            //fg_obj[1][3] = CreateBoneSphere();
            //fg_obj[1][3] = CreateBoneCapsule(Vec3d(0.2, 0.0, 0.0), Vec3d(0.0, 0.0, 0.0));
            fg_obj[1][3] = CreateBoneCapsuleRound(Vec3d(0.2, 0.0, 0.0), Vec3d(0.0, 0.0, 0.0), Quaterniond::Rot(Rad(30.0), 'x'));

            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj[0][3]);
            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj[1][3]);
 

            GetPHScene()->SetContactMode(fg_obj[0][3], PHSceneDesc::MODE_NONE);
            GetPHScene()->SetContactMode(fg_obj[1][3], PHSceneDesc::MODE_NONE);
            GetPHScene()->SetContactMode(fg_obj[0][3],cube, PHSceneDesc::MODE_LCP);
            GetPHScene()->SetContactMode(fg_obj[1][3],cube, PHSceneDesc::MODE_LCP);

            //virtual coupling
            fg_joint_vc[0][3] = GetPHScene()->CreateJoint(fg_base[0][3], fg_obj[0][3], Springdesc)->Cast();
            fg_joint_vc[1][3] = GetPHScene()->CreateJoint(fg_base[1][3], fg_obj[1][3], Springdesc)->Cast();

            //初期位置
            fg_base[0][3]->SetFramePosition(Vec3d(0.0, 0.2, 1.0));
            fg_base[1][3]->SetFramePosition(Vec3d(0.0, 0.2, -1.0));

            fg_obj[0][3]->SetFramePosition(Vec3d(0.0, 0.2, 1.0));
            fg_obj[1][3]->SetFramePosition(Vec3d(0.0, 0.2, -1.0));

        }
        if (type == 5) {

            //interface object
            fg_base[0][3] = CreateBone_base();
            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_base[0][3]);
            fg_base[1][3] = CreateBone_base();
            GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_base[1][3]);

            GetPHScene()->SetContactMode(fg_base[0][3], PHSceneDesc::MODE_NONE);
            GetPHScene()->SetContactMode(fg_base[1][3], PHSceneDesc::MODE_NONE);

            //tool object

            fg_obj[0][3] = CreateBoneSphere();
            fg_obj[1][3] = CreateBoneSphere();

            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj[0][3]);
            GetFWScene()->SetSolidMaterial(GRRenderIf::BLUE, fg_obj[1][3]);

            GetPHScene()->SetContactMode(fg_obj[0][3], PHSceneDesc::MODE_NONE);
            GetPHScene()->SetContactMode(fg_obj[1][3], PHSceneDesc::MODE_NONE);
            GetPHScene()->SetContactMode(fg_obj[0][3], cube, PHSceneDesc::MODE_LCP);
            GetPHScene()->SetContactMode(fg_obj[1][3], cube, PHSceneDesc::MODE_LCP);

            //virtual coupling
            fg_joint_vc[0][3] = GetPHScene()->CreateJoint(fg_base[0][3], fg_obj[0][3], Springdesc)->Cast();
            fg_joint_vc[1][3] = GetPHScene()->CreateJoint(fg_base[1][3], fg_obj[1][3], Springdesc)->Cast();

        }
    }

    // タイマコールバック関数．タイマ周期で呼ばれる
    virtual void OnStep() {
        // GetSdk()->SaveScene("test.spr", NULL, FIFileSprIf::GetIfInfoStatic());

        SampleApp::OnStep();

        // 床を揺らす
        if (soFloor && floorShakeAmplitude) {
            double time = GetFWScene()->GetPHScene()->GetCount() * GetFWScene()->GetPHScene()->GetTimeStep();
            double omega = 2.0 * M_PI;
            soFloor->SetFramePosition(Vec3d(floorShakeAmplitude * sin(time * omega), 0, 0));
            soFloor->SetVelocity(Vec3d(floorShakeAmplitude * omega * cos(time * omega), 0, 0));
        }

        if (type == 4) {
            Vec3d cube_pos;
            Vec3d fg_pos[2];
            Vec3d fg_obj_pos[2];


            //親指の指先の速度と角速度の測定とcsvファイルへの書き込み
            Vec3d fg_v;
            Vec3d fg_a_v;
            fg_v = fg_obj[0][3]->GetVelocity();
            fg_a_v = fg_obj[0][3]->GetAngularVelocity();

            std::string csv_filename = "test.csv";
            std::ofstream ofs_csv_file;
            ofs_csv_file.open(csv_filename, std::ios::app);
            ofs_csv_file << fg_v.x << ',' << fg_v.y << ',' << fg_v.z << ',' << fg_a_v.x << ',' << fg_a_v.y << ',' << fg_a_v.z << endl;



            //cubeのpositionに向かって二つの剛体が進むようになっている
            cube_pos = cube->GetFramePosition();

            fg_pos[0] = fg_base[0][3]->GetFramePosition();
            fg_pos[1] = fg_base[1][3]->GetFramePosition();

            fg_base[0][3]->SetFramePosition(fg_pos[0] + (cube_pos - fg_pos[0]) / jointLength(cube_pos, fg_pos[0]) * 0.001);
            fg_base[1][3]->SetFramePosition(fg_pos[1] + (cube_pos - fg_pos[1]) / jointLength(cube_pos, fg_pos[1]) * 0.001);


            //指のインターフェース側のsolidとツール側のsolidが0.05以上離れたら把持したとみなして持ち上げ、0.65の高さになったら落とす
            fg_obj_pos[0] = fg_obj[0][3]->GetFramePosition();
            if (jointLength(fg_obj_pos[0],fg_pos[0])>0.05) {
                if (fg_pos[0].y < 0.7) {
                    fg_base[0][3]->SetFramePosition(Vec3d(fg_pos[0].x, fg_pos[0].y + 0.001, fg_pos[0].z));
                    fg_base[1][3]->SetFramePosition(Vec3d(fg_pos[1].x, fg_pos[1].y + 0.001, fg_pos[1].z));
                }
            }
            if (fg_pos[0].y >= 0.65) {
                fg_base[0][3]->SetFramePosition(Vec3d(fg_pos[0].x, fg_pos[0].y , fg_pos[0].z+0.001));
                fg_base[1][3]->SetFramePosition(Vec3d(fg_pos[1].x, fg_pos[1].y , fg_pos[1].z-0.001));
                BuildScene();
            }
        }
        else {
            frame = GetFrame();

            if (frame && (frame->tracking_frame_id > lastFrameID)) {
                lastFrameID = frame->tracking_frame_id;
                //printf("Frame %lli with %i hands.\n", (long long int)frame->tracking_frame_id, frame->nHands);
                for (uint32_t h = 0; h < frame->nHands; h++) {
                    LEAP_HAND* hand = &frame->pHands[0];


                    if (type == 0) {
                        //
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
                    }

                    else if (type == 1) {

                        for (int i = 0; i < 5; i++) {
                            for (int j = 0; j < 4; j++) {
                                if (!(i == 0 && j == 0)) {
                                    fg_pos[i][j][0] = vecvec3_3(hand->digits[i].bones[j].prev_joint);
                                    fg_pos[i][j][1] = vecvec3_3(hand->digits[i].bones[j].next_joint);
                                    fg_ori[i][j] = quaquad(hand->digits[i].bones[j].rotation);
                                    fg_ori[i][j] = x90 * fg_ori[i][j];
                                    fg_ori[i][j] = y180 * fg_ori[i][j];

                                    fg_base_slide[i][j][0]->SetFramePosition(fg_pos[i][j][0]);
                                    fg_base_slide[i][j][1]->SetFramePosition(fg_pos[i][j][1]);
                                    fg_base_slide[i][j][0]->SetOrientation(fg_ori[i][j]);
                                    fg_base_slide[i][j][1]->SetOrientation(fg_ori[i][j]);

                                }
                            }
                            PHSolidPairForLCPIf* solidpair;
                            bool swaped;
                            int state;
                            state = 0;
                            solidpair = GetPHScene()->GetSolidPair(fg_obj_slide[0][3][0], cube, swaped);
                            if (solidpair && solidpair->GetSolid(0)->NShape() && solidpair->GetSolid(1)->NShape()) {
                                state = solidpair->GetContactState(0, 0);
                            }
                            clock_t start, end;
                            if (state == 1 || state == 2) {
                                start = clock();
                            }


                            //成功or失敗したらcubeを落とす
                            Vec3d cube_pos;
                            cube_pos = cube->GetFramePosition();
                            if (sorf == 0 && cube_pos.x > 0.5 && cube_pos.x<0.65 && cube_pos.z>-0.075 && cube_pos.z<0.075 && cube_pos.y>-2.0 && cube_pos.y < -1.0) {
                                end = clock();
                                std::ofstream writing_file;
                                std::string filename = "sample.txt";
                                writing_file.open(filename, std::ios::app);
                                //std::string writing_text = "test";
                                writing_file << ((float)end - start) / CLOCKS_PER_SEC << std::endl;
                                writing_file.close();
                                sorf = 1;
                            }
                            //if () {
                            //    sorf = 2;
                            //}
                            if (sorf == 1 || sorf == 2) {
                                Vec3d v, w(0.0, 0.0, 0.2), p(0.5, 10, 0.0);
                                static Quaterniond q = Quaterniond::Rot(Rad(0.0), 'y');
                                q = Quaterniond::Rot(Rad(90), 'y') * q;
                                Drop(SHAPE_BOX, GRRenderIf::RED, v, w, p, q);
                                sorf = 0;
                            }

                        }



                        //PHSolidPairForLCPIf* solidpair;
                        //int state;
                        //bool swaped;
                        //solidpair =GetPHScene()->GetSolidPair(fg_base_slide[2][2][0], fg_base_slide[2][2][1], swaped);
                        //solidpair->GetContactState(0, 0);
                        //cout<<fg_base_slide[2][2][0]->GetFramePosition()<<endl;
                        //if () {
                        //    for (int i = 0; i < 5; i++) {

                        //    }
                        //}
                    }
                    else if (type == 2) {

                        for (int i = 0; i < 5; i++) {
                            for (int j = 0; j < 4; j++) {
                                if (!(i == 0 && j == 0)) {
                                    fg_pos[i][j][0] = vecvec3_3(hand->digits[i].bones[j].prev_joint);
                                    fg_pos[i][j][1] = vecvec3_3(hand->digits[i].bones[j].next_joint);
                                    fg_ori[i][j] = quaquad(hand->digits[i].bones[j].rotation);
                                    fg_ori[i][j] = x90 * fg_ori[i][j];
                                    fg_ori[i][j] = y180 * fg_ori[i][j];

                                    fg_base_slide[i][j][0]->SetFramePosition(fg_pos[i][j][0]);
                                    fg_base_slide[i][j][1]->SetFramePosition(fg_pos[i][j][1]);
                                    fg_base_slide[i][j][0]->SetOrientation(fg_ori[i][j]);
                                    fg_base_slide[i][j][1]->SetOrientation(fg_ori[i][j]);

                                }
                            }
                            PHSolidPairForLCPIf* solidpair;
                            bool swaped;
                            int state;
                            solidpair = GetPHScene()->GetSolidPair(fg_obj_slide[0][3][0], cube, swaped);
                            //state=solidpair->GetContactState(0, 0);
                            //cout << state << endl();
                            clock_t start, end;
                            //if (state == 1 || state == 2) {
                            //    start = clock();
                            //}


                            //成功or失敗したらcubeを落とす
                            Vec3d cube_pos;
                            cube_pos = cube->GetFramePosition();
                            if (sorf == 0 && cube_pos.x > 0.5 && cube_pos.x<0.65 && cube_pos.z>-0.075 && cube_pos.z<0.075 && cube_pos.y>-2.0 && cube_pos.y < -1.0) {
                                end = clock();
                                std::ofstream writing_file;
                                std::string filename = "sample.txt";
                                writing_file.open(filename, std::ios::app);
                                //std::string writing_text = "test";
                                writing_file << ((float)end - start) / CLOCKS_PER_SEC << std::endl;
                                writing_file.close();
                                sorf = 1;
                            }
                            //if () {
                            //    sorf = 2;
                            //}
                            if (sorf == 1 || sorf == 2) {
                                Vec3d v, w(0.0, 0.0, 0.2), p(0.5, 10, 0.0);
                                static Quaterniond q = Quaterniond::Rot(Rad(0.0), 'y');
                                q = Quaterniond::Rot(Rad(90), 'y') * q;
                                Drop(SHAPE_BOX, GRRenderIf::RED, v, w, p, q);
                                sorf = 0;
                            }

                        }



                        //PHSolidPairForLCPIf* solidpair;
                        //int state;
                        //bool swaped;
                        //solidpair =GetPHScene()->GetSolidPair(fg_base_slide[2][2][0], fg_base_slide[2][2][1], swaped);
                        //solidpair->GetContactState(0, 0);
                        //cout<<fg_base_slide[2][2][0]->GetFramePosition()<<endl;
                        //if () {
                        //    for (int i = 0; i < 5; i++) {

                        //    }
                        //}
                    }
                    else if (type == 3) {
                        
                        for (int i = 0; i < 5; i++) {
                            for (int j = 0; j < 4; j++) {
                                if (!(i == 0 && j == 0)) {
                                    fg_pos[i][j][0] = vecvec3_3(hand->digits[i].bones[j].prev_joint);
                                    fg_pos[i][j][1] = vecvec3_3(hand->digits[i].bones[j].next_joint);
                                    fg_ori[i][j] = quaquad(hand->digits[i].bones[j].rotation);
                                    fg_ori[i][j] = x90 * fg_ori[i][j];
                                    fg_ori[i][j] = y180 * fg_ori[i][j];

                                    fg_base_slide[i][j][0]->SetFramePosition(fg_pos[i][j][0]);
                                    fg_base_slide[i][j][1]->SetFramePosition(fg_pos[i][j][1]);
                                    fg_base_slide[i][j][0]->SetOrientation(fg_ori[i][j]);
                                    fg_base_slide[i][j][1]->SetOrientation(fg_ori[i][j]);

                                    fg_joint_vc_slide[i][j][0]->SetPlugPose(Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 2.0));
                                    fg_joint_vc_slide[i][j][1]->SetPlugPose(Vec3d(0.0, 0.0, jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 2.0));

                                }
                            }
                        }
                        for (int i = 0; i < 2; i++) {
                            for (int j = 0; j < 3; j++) {
                                if (!(i == 0 && j == 0)) {
                                    fg_joint[i][j]->SetSocketPose(Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1]) / 2.0));
                                    fg_joint[i][j]->SetPlugPose(Vec3d(0.0, 0.0, jointLength(fg_pos[i][j + 1][0], fg_pos[i][j + 1][1]) / 2.0));
                                }
                            }
                        }

                        PHSolidPairForLCPIf* solidpair;
                        bool swaped;
                        int state;
                        solidpair = GetPHScene()->GetSolidPair(fg_obj[0][3], cube, swaped);
                        //state=solidpair->GetContactState(0, 0);
                        //cout << state << endl();
                        clock_t start, end;
                        //if (state == 1 || state == 2) {
                        //    start = clock();
                        //}

                        Vec3d fg_v;
                        Vec3d fg_a_v;
                        fg_v = fg_obj[0][3]->GetVelocity();
                        fg_a_v = fg_obj[0][3]->GetAngularVelocity();

                        std::string csv_filename = "test.csv";
                        std::ofstream ofs_csv_file;
                        ofs_csv_file.open(csv_filename, std::ios::app);
                        ofs_csv_file << fg_v.x << ',' << fg_v.y << ',' << fg_v.z << ',' << fg_a_v.x <<','<< fg_a_v.y <<','<< fg_a_v.z << endl;


                        //成功or失敗したらcubeを落とす
                        Vec3d cube_pos;
                        cube_pos = cube->GetFramePosition();
                        if (sorf == 0 && cube_pos.x > 0.5 && cube_pos.x<0.65 && cube_pos.z>-0.075 && cube_pos.z<0.075 && cube_pos.y>-2.0 && cube_pos.y < -1.0) {
                            end = clock();
                            std::ofstream writing_file;
                            std::string filename = "sample.txt";
                            writing_file.open(filename, std::ios::app);
                            //std::string writing_text = "test";
                            writing_file << ((float)end - start) / CLOCKS_PER_SEC << std::endl;
                            writing_file.close();
                            sorf = 1;
                        }
                        //if () {
                        //    sorf = 2;
                        //}
                        if (sorf == 1 || sorf == 2) {
                            Vec3d v, w(0.0, 0.0, 0.2), p(0.5, 10, 0.0);
                            static Quaterniond q = Quaterniond::Rot(Rad(0.0), 'y');
                            q = Quaterniond::Rot(Rad(90), 'y') * q;
                            Drop(SHAPE_BOX, GRRenderIf::RED, v, w, p, q);
                            sorf = 0;
                        }
                    }
                    else if (type == 5) {
                        // finger position from leapmotion
                        fg_pos[0][3][1] = vecvec3_3(hand->digits[0].bones[3].next_joint);
                        fg_pos[1][3][1] = vecvec3_3(hand->digits[1].bones[3].next_joint);

                        // finger rotation from leapmotion
                        fg_ori[0][3] = quaquad(hand->digits[0].bones[3].rotation);
                        fg_ori[0][3] = x90 * fg_ori[0][3];
                        fg_ori[0][3] = y180 * fg_ori[0][3];
                        fg_ori[1][3] = quaquad(hand->digits[1].bones[3].rotation);
                        fg_ori[1][3] = x90 * fg_ori[1][3];
                        fg_ori[1][3] = y180 * fg_ori[1][3];

                        // Update position and orientation
                        fg_base[0][3]->SetFramePosition(fg_pos[0][3][1]);
                        fg_base[1][3]->SetFramePosition(fg_pos[1][3][1]);
                        fg_base[0][3]->SetOrientation(fg_ori[0][3]);
                        fg_base[1][3]->SetOrientation(fg_ori[1][3]);

                        // If the cube falls into the hole, set again
                        Vec3d cube_pos;
                        cube_pos = cube->GetFramePosition();
                        if (sorf == 0 && cube_pos.x > 0.5 && cube_pos.x<0.65 && cube_pos.z>-0.075 && cube_pos.z<0.075 && cube_pos.y>-2.0 && cube_pos.y < -1.0) {
                            end = clock();
                            std::ofstream writing_file;
                            std::string filename = "sample.txt";
                            writing_file.open(filename, std::ios::app);
                            //std::string writing_text = "test";
                            writing_file << ((float)end - start) / CLOCKS_PER_SEC << std::endl;
                            writing_file.close();
                            sorf = 1;
                        }
                        if (sorf == 1 || sorf == 2) {
                            Vec3d v, w(0.0, 0.0, 0.2), p(0.5, 10, 0.0);
                            static Quaterniond q = Quaterniond::Rot(Rad(0.0), 'y');
                            q = Quaterniond::Rot(Rad(90), 'y') * q;
                            Drop(SHAPE_BOX, GRRenderIf::RED, v, w, p, q);
                            sorf = 0;
                        }
                    }
                }
            }
        }
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
                //start = clock();
            }
            if (id == ID_CAPSULE) {
                Drop(SHAPE_CAPSULE, GRRenderIf::GREEN, v, w, p, q);
                message = "capsule dropped.";
                //end = clock();
                //cout << ((float)end - start) / CLOCKS_PER_SEC << endl;
            }
            if (id == ID_ROUNDCONE) {
                Drop(SHAPE_ROUNDCONE, GRRenderIf::BLUE, v, w, p, q);
                message = "round cone dropped.";

            }
            if (id == ID_SPHERE) {
                Drop(SHAPE_SPHERE, GRRenderIf::YELLOW, v, w, p, q);
                message = "sphere dropped.";

            }
            if (id == ID_ELLIPSOID) {
                Drop(SHAPE_ELLIPSOID, GRRenderIf::LIGHTGREEN, v, w, p, q);
                message = "sphere dropped.";
            }
            if (id == ID_ROCK) {
                Drop(SHAPE_ROCK, GRRenderIf::ORANGE, v, w, p, q);
                message = "random polyhedron dropped.";

            }
            if (id == ID_BLOCK) {
                Drop(SHAPE_BLOCK, GRRenderIf::CYAN, v, w, p, q);
                message = "composite block dropped.";

            }

            if (id == ID_SHAKE) {
                std::cout << "F: shake floor." << std::endl;
                if (floorShakeAmplitude == 0.0) {
                  floorShakeAmplitude = 2.5;
                  message = "floor shaken.";
                }
                else {
                  floorShakeAmplitude = 0;
                  soFloor->SetFramePosition(Vec3d(0, 0, 0));
                  soFloor->SetVelocity(Vec3d(0, 0, 0));
                  message = "floor stopped.";
                }

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
