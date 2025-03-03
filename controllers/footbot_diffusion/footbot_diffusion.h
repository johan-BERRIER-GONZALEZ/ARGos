#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

using namespace argos;

class CFootBotDiffusion : public CCI_Controller {

public:

   CFootBotDiffusion();
   virtual ~CFootBotDiffusion() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotProximitySensor* m_pcProximity;
   CCI_PositioningSensor* m_pcCompass;

   Real m_fWheelVelocity;
   Real m_fStopDistance;
   Real m_fTheta;
   Real m_fTimeStep;
   bool m_bAvoidingObstacle;
   bool m_bTurning;
   CRadians m_fThetaTarget;

   Real GetFrontMinObstacleDistance();
   Real GetHeadingDegrees() const;
   Real Sawtooth(CRadians angle);

};

#endif

