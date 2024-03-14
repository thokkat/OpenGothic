#include "cscamera.h"

#include <Tempest/Log>
#include <cassert>

#include "gothic.h"

using namespace Tempest;

CsCamera::CsCamera(Vob* parent, World& world, const zenkit::VCutsceneCamera& cam, Flags flags)
  :AbstractTrigger(parent,world,cam,flags) {
  if(cam.position_count<1 || (cam.position_count<2 && cam.target_count<1) || cam.total_duration<=0)
    return;

  if(cam.trajectory_for==zenkit::CameraCoordinateReference::OBJECT || cam.target_trajectory_for==zenkit::CameraCoordinateReference::OBJECT) {
    Log::d("Object camera not implemented, \"", name() , "\"");
    return;
    }

  duration      = uint64_t(cam.total_duration * 1000.f);
  delay         = uint64_t(cam.auto_untrigger_last_delay * 1000.f);
  autoUntrigger = cam.auto_untrigger_last;
  playerMovable = cam.auto_player_movable;

  for(auto& f : cam.trajectory_frames) {
    KeyFrame kF;
    kF.position   = Vec3(f->original_pose[3][0],f->original_pose[3][1],f->original_pose[3][2]);
    kF.time       = f->time;
    kF.motionType = f->motion_type;
    posSpline.keyframes.push_back(kF);
    }

  for(auto& f : cam.target_frames) {
    KeyFrame kF;
    kF.position   = Vec3(f->original_pose[3][0],f->original_pose[3][1],f->original_pose[3][2]);
    kF.time       = f->time;
    kF.motionType = f->motion_type;
    targetSpline.keyframes.push_back(kF);
    }

  for(auto spl : {&posSpline,&targetSpline}) {
    if(spl->keyframes.front().time!=0)
      Log::e("CsCamera: \"",cam.vob_name,"\" - invalid first frame");
    // EVT_TPL_SLEEPERCAM_CAMERA_01 has a duration less than last target frame's time
    if(spl->keyframes.back().time!=cam.total_duration)
      spl->keyframes.back().time = cam.total_duration;
    }
  }

bool CsCamera::isPlayerMovable() const {
  return playerMovable;
  }

void CsCamera::onTrigger(const TriggerEvent& evt) {
  if(isTicksEnabled() || posSpline.size()==0)
    return;

  if(auto cs = world.currentCs())
    cs->onUntrigger(evt);

  auto& camera = world.gameSession().camera();
  if(!camera.isCutscene()) {
    camera.reset();
    camera.setMode(Camera::Mode::Cutscene);
    }

  time                 = 0;
  godMode              = Gothic::inst().isGodMode();
  Gothic::inst().setGodMode(true);
  world.setCurrentCs(this);
  enableTicks();
  }

void CsCamera::onUntrigger(const TriggerEvent& evt) {
  if(!isTicksEnabled())
    return;
  disableTicks();
  if(world.currentCs()!=this)
    return;

  world.setCurrentCs(nullptr);
  Gothic::inst().setGodMode(godMode);

  auto& camera = world.gameSession().camera();
  camera.setMode(Camera::Mode::Normal);
  camera.reset();
  }

void CsCamera::tick(uint64_t dt) {
  time += dt;

  if(time>duration+delay && (autoUntrigger || vobName=="TIMEDEMO")) {
    TriggerEvent e("","",TriggerEvent::T_Untrigger);
    onUntrigger(e);
    return;
    }

  if(time>duration)
    return;

  auto& camera = world.gameSession().camera();
  if(camera.isCutscene()) {
    auto cPos = position();
    camera.setPosition(cPos);
    camera.setSpin(spin(cPos));
    }
  }

Vec3 CsCamera::position() const {
  return posSpline.position(time);
  }

PointF CsCamera::spin(Tempest::Vec3& d) const {
  if(targetSpline.size()==0)
    d = d - Gothic::inst().camera()->destPosition(); else
    d = targetSpline.position(time) - d;

  float k     = 180.f/float(M_PI);
  float spinX = k * std::asin(d.y/d.length());
  float spinY = -90;
  if(d.x!=0.f || d.z!=0.f)
    spinY = 90 + k * std::atan2(d.z,d.x);
  return {-spinX,spinY};
  }

Vec3 CsCamera::Trajectory::position(uint64_t time) const {
  float t = applyMotionScaling(time);
  for(size_t i = 1;i<size();++i) {
    auto& kF2 = keyframes[i];
    if(kF2.time>t) {
      auto& kF0 = i==1 ? keyframes[i-1] : keyframes[i-2];
      auto& kF1 = keyframes[i-1];
      auto& kF3 = i+1==size() ? keyframes[i] : keyframes[i+1];
      float dt  = kF2.time-kF1.time;
      Vec3  dd  = (kF2.position-kF0.position) * dt/(kF2.time-kF0.time);
      Vec3  sd  = (kF3.position-kF1.position) * dt/(kF3.time-kF1.time);
      Vec3  a   =  2*kF1.position - 2*kF2.position +   dd + sd;
      Vec3  b   = -3*kF1.position + 3*kF2.position - 2*dd - sd;
      float u   = (t-kF1.time) / dt;
      return kF1.position + ((a*u + b)*u + dd)*u;
      }
    }
  return keyframes.back().position;
  }

float CsCamera::Trajectory::applyMotionScaling(uint64_t time) const {
  float t   = float(time)/1000.f;
  auto  kF1 = &keyframes.front();
  auto  kF2 = &keyframes.back();
  for(auto& kF:keyframes) {
    if(kF.motionType==CameraMotion::UNDEFINED || kF.motionType==CameraMotion::CUSTOM || kF.motionType==CameraMotion::SMOOTH)
      continue;
    if(kF.time>t) {
      kF2 = &kF;
      break;
      } else {
      kF1 = &kF;
      }
    }

  if(kF1==kF2 || kF2->motionType==CameraMotion::STEP)
    return kF2->time;
  float d1      = kF1->speed();
  float d2      = kF2->speed();
  float dt      = kF2->time - kF1->time;
  float u       = (t - kF1->time) / dt;
  float tScaled = (((d1 + d2 - 2)*u -2*d1 - d2 + 3)*u + d1)*u*dt;
  return kF1->time + tScaled;
  }

float CsCamera::KeyFrame::speed() const {
  switch(motionType) {
    case CameraMotion::SLOW:
      return 0;
    case CameraMotion::CUSTOM:
    case CameraMotion::LINEAR:
    case CameraMotion::SMOOTH:
    case CameraMotion::STEP:
    case CameraMotion::UNDEFINED:
      return 1;
    case CameraMotion::FAST:
      return 2;
    }
  return 1;
  }
