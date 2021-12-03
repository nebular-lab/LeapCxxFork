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
#include "FWFileLoaderSample.h"
#include "../src/Samples/SampleApp.h"
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

FWFileLoaderSample::FWFileLoaderSample() {
#ifdef USE_SPRFILE
    fileName = "./file/scene.spr";	// sprファイル
#else
    fileName = "./file/scene.x";		// xファイル
#endif
}
void FWFileLoaderSample::Init(int argc, char* argv[]) {
    CreateSdk();			// SDKの作成
    UTRef<ImportIf> import = GetSdk()->GetFISdk()->CreateImport();
    GetSdk()->LoadScene(fileName, import);			// ファイルのロード
    GetSdk()->SaveScene("save.spr", import);		// ファイルのセーブテスト

}

void FWFileLoaderSample::InitCameraView() {
    Vec3d pos = Vec3d(-0.978414, 11.5185, 24.4473);		// カメラ初期位置
    GetCurrentWin()->GetTrackball()->SetPosition(pos);	// カメラ初期位置の設定
}

void FWFileLoaderSample::Reset() {
    GetSdk()->Clear();
    GetSdk()->LoadScene(fileName);
    GetCurrentWin()->SetScene(GetSdk()->GetScene());
}


void FWFileLoaderSample::Keyboard(int key, int x, int y) {
    switch (key) {
    case ESC:
    case 'q':
        // アプリケーションの終了
        exit(0);
        break;
    case 'r':
        // ファイルの再読み込み
        Reset();
        break;
    case 'w':
        // カメラ位置の初期化
        InitCameraView();
        break;
    case 'd':
    {
        // デバック表示
        static bool bDebug = GetSdk()->GetDebugMode();
        if (bDebug)	bDebug = false;
        else		bDebug = true;
        GetSdk()->SetDebugMode(bDebug);
        DSTR << "Debug Mode " << bDebug << std::endl;
        //DSTR << "CameraPosition" << std::endl;
        //DSTR << GetCurrentWin()->GetTrackball()->GetPosition() << std::endl;
    }
    break;
    default:
        break;
    }
}


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

    double scale = 3.0;
    double move_scale = 1.5;
    float friction = 1.0;
    float density = 1000.0;

    float cube_density = 2500.0;
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



    PHSpringDesc oyahitodesc;

    PHSolidIf* cylinder;






    PHSolidIf* jenga1;
    PHSolidIf* jenga2;
    PHSolidIf* jenga3;
    PHSolidIf* jenga4;
    PHSolidIf* jenga5;
    PHSolidIf* jenga6;
    PHSolidIf* jenga7;
    PHSolidIf* jenga8;
    PHSolidIf* jenga9;






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
    Vec3d palm_v;

    

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
        type = 1;

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
    PHSolidIf* CreateFloor(bool bWall) {
        PHSolidIf* soFloor = GetPHScene()->CreateSolid();
        soFloor->SetName("soFloor");
        soFloor->SetDynamical(false);
        double from_center;
        double width;
        from_center = 0.5;
        width = 0.15;
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
        GetFWScene()->SetSolidMaterial(GRRenderIf::GRAY, soFloor);
        soFloor->CompInertia();

        return soFloor;
    }
    // //床の作成
    //PHSolidIf* CreateFloor(bool bWall) {

    //    PHSolidIf* soFloor = GetPHScene()->CreateSolid();
    //    soFloor->SetName("soFloor");
    //    soFloor->SetDynamical(false);

    //    CDBoxDesc bd1;
    //    CDBoxDesc bd2;
    //    CDBoxDesc bd3;
    //    CDBoxDesc bd4;
    //    bd1.boxsize = Vec3d(11.0, 1.0, 5.0)*0.001*scale*100;
    //    bd2.boxsize = Vec3d(5.0, 1.0, 6.0) * 0.001 * scale*100;
    //    bd3.boxsize = Vec3d(10.0, 1.0, 6.0) * 0.001 * scale*100;
    //    bd4.boxsize = Vec3d(6.0, 1.0, 5.0) * 0.001 * scale*100;

    //    shapeFloor= GetSdk()->GetPHSdk()->CreateShape(bd1)->Cast();
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->SetShapePose(0, Posed::Trn(-2.5, 0, 3.0) * 0.001 * scale*100);

    //    shapeFloor = GetSdk()->GetPHSdk()->CreateShape(bd2)->Cast();
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->SetShapePose(1, Posed::Trn(5.5, 0, 2.5) * 0.001 * scale*100);

    //    shapeFloor = GetSdk()->GetPHSdk()->CreateShape(bd3)->Cast();
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->SetShapePose(2, Posed::Trn(-3.0, 0, -2.5) * 0.001 * scale*100);

    //    shapeFloor = GetSdk()->GetPHSdk()->CreateShape(bd4)->Cast();
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->SetShapePose(3, Posed::Trn(5.0, 0, -3.0) * 0.001 * scale*100);

    //    soFloor->SetShapePose(0, Posed::Trn(22.0, -shapeFloor->GetBoxSize().y / 2+0.10, 0));
    //    if (bWall) {
    //        soFloor->AddShape(shapeWallZ);
    //        soFloor->AddShape(shapeWallX);
    //        soFloor->AddShape(shapeWallZ);
    //        soFloor->AddShape(shapeWallX);
    //        double y = shapeWallZ->GetBoxSize().y / 2 - shapeFloor->GetBoxSize().y;
    //        soFloor->SetShapePose(1, Posed::Trn(-(shapeFloor->GetBoxSize().x + shapeWallZ->GetBoxSize().x) / 2, y, 0));
    //        soFloor->SetShapePose(2, Posed::Trn(0, y, -(shapeFloor->GetBoxSize().z + shapeWallX->GetBoxSize().z) / 2));
    //        soFloor->SetShapePose(3, Posed::Trn((shapeFloor->GetBoxSize().x + shapeWallZ->GetBoxSize().x) / 2, y, 0));
    //        soFloor->SetShapePose(4, Posed::Trn(0, y, (shapeFloor->GetBoxSize().z + shapeWallX->GetBoxSize().z) / 2));
    //    }
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->AddShape(shapeFloor);
    //    soFloor->SetShapePose(5, Posed::Trn(-22.0, -shapeFloor->GetBoxSize().y / 2, 0));
    //    soFloor->SetShapePose(6, Posed::Trn(0, -shapeFloor->GetBoxSize().y / 2, -22.0));
    //    soFloor->SetShapePose(7, Posed::Trn(0, -shapeFloor->GetBoxSize().y / 2, 42.3));

    //    GetFWScene()->SetSolidMaterial(GRRenderIf::GRAY, soFloor);
    //    soFloor->CompInertia();

    //    return soFloor;
    //}

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

        soBone->CompInertia();
        //soBone->SetMass(1.0f);
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
    PHSolidIf* CreateBone2() {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDSphereDesc sd;
        sd.radius = 3.0f / 1000.0f * (float)scale;

        shapeSphere = GetSdk()->GetPHSdk()->CreateShape(sd)->Cast();

        soBone->SetDynamical(false);


        soBone->AddShape(shapeSphere);


        return soBone;
    }
    //fg_obj用のsolid
    PHSolidIf* CreateBone3(Vec3d p, Vec3d n) {
        PHSolidIf* soBone = GetPHScene()->CreateSolid();
        CDCapsuleDesc cd;

        //1.0->2.0に修正
        cd.radius = 7.5f/1000.0f * (float)scale;
        cd.length = (float)sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y) + (p.z - n.z) * (p.z - n.z));


        shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
        shapeCapsule->SetDensity(density);
        shapeCapsule->SetStaticFriction(100.0f);
        shapeCapsule->SetDynamicFriction(100.0f);

        soBone->AddShape(shapeCapsule);

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
        PHSolidIf* solid = GetPHScene()->CreateSolid();
        // マテリアルを設定
        GetFWScene()->SetSolidMaterial(mat, solid);

        // 形状の割当て
        if (shape == SHAPE_BOX) {
            CDBoxDesc bd;
            bd.boxsize = Vec3d(1.5, 1.5, 1.5)/100.0 * scale;  //単位 m^3
            shapeBox->SetDynamicFriction(friction);
            shapeBox->SetStaticFriction(friction);
            shapeBox->SetDensity(cube_density);//単位 kg/m^3
            shapeBox = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
            solid->AddShape(shapeBox);
        }

        if (shape == SHAPE_CAPSULE) {
            CDCapsuleDesc cd;
            cd.radius=1.0f / 100.0f * (float)scale;
            cd.length = 10.0f / 100.0f * (float)scale;
            shapeCapsule->SetDynamicFriction(friction);
            shapeCapsule->SetStaticFriction(friction);
            shapeCapsule->SetDensity(cube_density);
            shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
            solid->AddShape(shapeCapsule);

        }

        if (shape == SHAPE_ROUNDCONE)
            solid->AddShape(shapeRoundCone);
        if (shape == SHAPE_SPHERE)
            solid->AddShape(shapeSphere);
        if (shape == SHAPE_ELLIPSOID){
            //solid->AddShape(shapeEllipsoid);
            CDBoxDesc bd;
            bd.boxsize = Vec3d(3.0, 15.0, 3.0) / 100.0 * scale;  //単位 m^3
            shapeBox->SetDynamicFriction(friction);
            shapeBox->SetStaticFriction(friction);
            shapeBox->SetDensity(cube_density);//単位 kg/m^3
            shapeBox = GetSdk()->GetPHSdk()->CreateShape(bd)->Cast();
            solid->AddShape(shapeBox);

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
            solid->AddShape(GetSdk()->GetPHSdk()->CreateShape(md));
        }
        if (shape == SHAPE_BLOCK) {
            for (int i = 0; i < 7; i++)
                solid->AddShape(shapeBox);
            Posed pose;
            pose.Pos() = ShapeScale() * Vec3d(3, 0, 0); solid->SetShapePose(1, pose);
            pose.Pos() = ShapeScale() * Vec3d(-3, 0, 0); solid->SetShapePose(2, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, 3, 0); solid->SetShapePose(3, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, -3, 0); solid->SetShapePose(4, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, 0, 3); solid->SetShapePose(5, pose);
            pose.Pos() = ShapeScale() * Vec3d(0, 0, -3); solid->SetShapePose(6, pose);
        }
        if (shape == SHAPE_COIN) {
            solid->AddShape(shapeCoin);
        }


        if (type == 1) {

            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {

                        GetPHScene()->SetContactMode(fg_base_slide[i][j][0], solid, PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_base_slide[i][j][1], solid, PHSceneDesc::MODE_NONE);


                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][0], solid, PHSceneDesc::MODE_LCP);
                        GetPHScene()->SetContactMode(fg_obj_slide[i][j][1], solid, PHSceneDesc::MODE_LCP);
                    }
                }
            }
            
        }
        solid->SetVelocity(v);
        solid->SetAngularVelocity(w);
        p = Vec3d(0.0, 0.5, 0.0);
        solid->SetFramePosition(p);
        solid->SetOrientation(q);
        solid->CompInertia();
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


        soFloor = CreateFloor(true);
        GetCurrentWin()->GetTrackball()->SetPosition(Vec3d(0.0,2.5,10.0));

        GetFWScene()->EnableRenderAxis(false, false, false);
        GetFWScene()->EnableRenderForce(false, false);
        printf("手をかざしてお待ちください。\n");

        int detect;
        LEAP_TRACKING_EVENT* frame;

        detect = 0;
        while (detect == 0) {
            frame = GetFrame();
            printf("%d\n", (int)frame->nHands);
            if ((int)frame->nHands > 0) {
                detect = 1;
            }
        }
        std::ofstream writing_file;
        std::string filename = "sample.txt";
        writing_file.open(filename, std::ios::app);
        std::string writing_text = "test";
        writing_file << writing_text << std::endl;
        writing_file.close();

        //LEAP_TRACKING_EVENT* frame = GetFrame();
        LEAP_HAND* hand = &frame->pHands[0];
        //printf("hand is detected\n");

        float tip_width[5] = { 0.0 };


        for (int i = 0; i < 5; i++) {
            tip_width[i] = hand->digits[i].distal.width;
            //tip_width[i] *= 1.5;
        }
        double spring;
        double damper;
        spring = 100.0;
        damper = 10.0;
        //Springdesc.spring = Vec3d(spring, spring, spring);
        //Springdesc.damper = Vec3d(damper, damper, damper);
        //Springdesc.springOri = spring;
        //Springdesc.damperOri = damper;
        if (type == 0) {
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

                        fg_base[i][j] = CreateBone2();
                        GetFWScene()->SetSolidMaterial(GRRenderIf::RED, fg_base[i][j]);
                        if (j == 3) {//指先
                            fg_obj[i][j] = CreateBone(tip_width[i]);
                        }
                        else {//指の骨
                            fg_obj[i][j] = CreateBone3(fg_pos[i][j][0], fg_pos[i][j][1]);
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
            //for (int i = 0; i < 5; i++) {
            //	for (int j = 1; j < 3; j++) {
            //		if (!(i == 0 && j == 0)) {
            //			fg_hinge_desc[i][j].poseSocket.Pos()= Vec3d(0.0, 0.0, -jointLength(fg_pos[i][j][0], fg_pos[i][j][1])) / 2.0;
            //			fg_hinge_desc[i][j].posePlug.Pos() = Vec3d(0.0, 0.0, jointLength(fg_pos[i][j + 1][0], fg_pos[i][j + 1][1])) / 2.0;
            //			fg_joint[i][j] = GetPHScene()->CreateJoint(fg_obj[i][j], fg_obj[i][j + 1], fg_hinge_desc[i][j])->Cast();
            //		}
            //	}
            //}
            //palm_middle_obj = CreatePalm_obj();
            //palm_middle_base = CreateBone2();
            //palm_middle_joint_vc= GetPHScene()->CreateJoint(palm_middle_base, palm_middle_obj, Springdesc)->Cast();


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
                        fg_base_slide[i][j][0] = CreateBone2();
                        fg_base_slide[i][j][1] = CreateBone2();
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][0]);
                        GetFWScene()->SetSolidMaterial(GRRenderIf::GREEN, fg_base_slide[i][j][1]);

                        GetPHScene()->SetContactMode(fg_base_slide[i][j][0], PHSceneDesc::MODE_NONE);
                        GetPHScene()->SetContactMode(fg_base_slide[i][j][1], PHSceneDesc::MODE_NONE);

                    }
                }
            }
            //palm_middle_base = CreateBone2();
            //Get

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
                        //printf("fg_obj_slide[%d][%d][0]:%p\n", i, j, &fg_obj_slide[i][j][0]);
                        //printf("fg_obj_slide[%d][%d][1]:%p\n", i, j, &fg_obj_slide[i][j][1]);


                        GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][0]);
                        GetFWScene()->SetSolidMaterial(GRRenderIf::YELLOW, fg_obj_slide[i][j][1]);


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
            //スライダージョイント
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    if (!(i == 0 && j == 0)) {
                        Sliderdesc.spring = 1000.0;
                        Sliderdesc.damper = 10.0;
                        fg_slider[i][j] = GetPHScene()->CreateJoint(fg_obj_slide[i][j][0], fg_obj_slide[i][j][1], Sliderdesc)->Cast();
                        fg_slider[i][j]->CreateLimit(slider_limit_desc);
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
            PHBallJointIf* fg_joint_palm[3][2];
            PHBallJointDesc palm_ball_desc[3][2];
            for (int i = 0; i < 3; i++) {
                palm_ball_desc[i][0].poseSocket.Pos() = Vec3d(0.0, 0.0, -jointLength(fg_pos[i+1][0][0], fg_pos[i+1][0][1]) / 4.0);
                palm_ball_desc[i][0].posePlug.Pos() = Vec3d(-8.5f / 1000.0f * (float)scale*2, 0.0, -jointLength(fg_pos[i + 2][0][0], fg_pos[i + 2][0][1]) / 4.0);// + fg_pos[i + 1][0][1] - fg_pos[i + 2][0][1]
                fg_joint_palm[i][0] = GetPHScene()->CreateJoint(fg_obj_slide[i+1][0][1], fg_obj_slide[i+2][0][1], palm_ball_desc[i][0])->Cast();
                
            }
        }
    }

    // タイマコールバック関数．タイマ周期で呼ばれる
    virtual void OnStep() {
        // GetSdk()->SaveScene("test.spr", NULL, FIFileSprIf::GetIfInfoStatic());

        SampleApp::OnStep();

        step_count++;
        printf("count_step=%d\n", step_count);



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
                leap_count++;
                //printf("hand=%d\n", (int)frame->nHands);
                printf("count_leap=%d\n",leap_count);
                palm_v = vecvec3_2(hand->palm.velocity);

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
                                //fg_base[i][j]->SetVelocity(palm_v);
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

                //for (int i = 1; i < 2; i++) {
                //    for (int j = 0; j < 4; j++) {
                //        if (!(i == 0 && j == 0)) {
                //            CDCapsuleDesc cd;
                //            cd.length = jointLength(fg_pos[i][j][0], fg_pos[i][j][1]);
                //            shapeCapsule = GetSdk()->GetPHSdk()->CreateShape(cd)->Cast();
                //            fg_obj[i][j]->AddShape(shapeCapsule);

                //            
                //        }
                //    }
                //}

                //for (int i = 1; i < 2; i++) {
                //    for (int j = 0; j < 4; j++) {
                //        if (!(i == 0 && j == 0)) {
                //            fg_pos[i][j][0] = vecvec3_2(hand->digits[i].bones[j].prev_joint);
                //            fg_pos[i][j][1] = vecvec3_2(hand->digits[i].bones[j].next_joint);
                //            printf("%f,", jointLength(fg_pos[i][j][0], fg_pos[i][j][1]));
                //        }
                //    }
                //}
                //printf("\n");


                //palm_middle_pos = vecvec3_3(hand->palm.position);
                //palm_middle_ori = quaquad(hand->palm.orientation);
                //palm_middle_ori = x90 * palm_middle_ori;
                //palm_middle_ori = y180 * palm_middle_ori;
                //palm_middle_base->SetFramePosition(palm_middle_pos);
                //palm_middle_base->SetOrientation(palm_middle_ori);
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
                //start = clock();
            }
            if (id == ID_CAPSULE) {
                Drop(SHAPE_CAPSULE, GRRenderIf::GREEN, v, w, p, q);
                message = "capsule dropped.";
                //end = clock();
                //cout << ((float)end - start) / CLOCKS_PER_SEC << endl;
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
