#pragma once

#include <zenkit/vobs/Camera.hh>

#include "abstracttrigger.h"

class World;

class CsCamera : public AbstractTrigger {
  public:
    CsCamera(Vob* parent, World& world, const zenkit::VCutsceneCamera& data, Flags flags);

    bool isPlayerMovable() const;

  private:
    using CameraMotion = zenkit::CameraMotion;

    struct KeyFrame {
      float         time       = 0;
      Tempest::Vec3 position   = {};
      CameraMotion  motionType = CameraMotion::SMOOTH;
      float speed() const;
      };

    struct Trajectory {
      std::vector<KeyFrame> keyframes;
      size_t size() const { return keyframes.size(); }
      auto   position(uint64_t time) const -> Tempest::Vec3;
      float  applyMotionScaling(uint64_t time) const;
      };

    void onTrigger(const TriggerEvent& evt) override;
    void onUntrigger(const TriggerEvent& evt) override;
    void tick(uint64_t dt) override;

    auto position() const -> Tempest::Vec3;
    auto spin(Tempest::Vec3& d) const -> Tempest::PointF;

    bool       godMode       = false;
    bool       playerMovable = false;
    bool       autoUntrigger = false;
    uint64_t   duration      = 0;
    uint64_t   delay         = 0;
    uint64_t   time          = 0;
    Trajectory posSpline     = {};
    Trajectory targetSpline  = {};
  };
