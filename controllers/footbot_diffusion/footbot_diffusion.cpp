#include "footbot_diffusion.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <cmath>
#include <cstdio>

CFootBotDiffusion::CFootBotDiffusion() :
    m_pcWheels(nullptr),
    m_pcProximity(nullptr),
    m_pcCompass(nullptr),
    m_fWheelVelocity(2.5f),
    m_fStopDistance(0.2f),
    m_bAvoidingObstacle(false),
    m_bTurning(false),
    m_fThetaTarget(0.0f) {}

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcCompass = GetSensor<CCI_PositioningSensor>("positioning");

    if (!m_pcCompass) {
        printf("\u274C Erreur : Capteur de boussole non initialisé !\n");
    }

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "stop_distance", m_fStopDistance, m_fStopDistance);
}

Real CFootBotDiffusion::Sawtooth(CRadians angle) {
    return atan2(sin(angle.GetValue()), cos(angle.GetValue()));
}

Real CFootBotDiffusion::GetFrontMinObstacleDistance() {
    const auto& tProxReads = m_pcProximity->GetReadings();
    Real min_distance = std::numeric_limits<Real>::infinity();

    for (const auto& reading : tProxReads) {
        CRadians sensor_angle = reading.Angle;
        if (sensor_angle.GetValue() > -CRadians(95.0 * ARGOS_PI / 180.0).GetValue() &&
            sensor_angle.GetValue() < CRadians(95.0 * ARGOS_PI / 180.0).GetValue()) {
            
            Real distance = 0.23f - 0.23f * reading.Value;
            min_distance = std::min(min_distance, distance);
        }
    }
    return min_distance;
}

Real CFootBotDiffusion::GetHeadingDegrees() const {
    return ToDegrees(CRadians(m_fTheta)).GetValue();
}

void CFootBotDiffusion::ControlStep() {
    if (!m_pcCompass) {
        printf("\u274C Erreur : Capteur de boussole non disponible !\n");
        return;
    }

    const auto& sPosition = m_pcCompass->GetReading();
    CRadians cYaw, cPitch, cRoll;
    sPosition.Orientation.ToEulerAngles(cYaw, cPitch, cRoll);
    m_fTheta = cYaw.GetValue();

    printf("\U0001F4E1 Cap actuel : %.2f rad (%.2f°)\n", m_fTheta, GetHeadingDegrees());

    Real min_distance = GetFrontMinObstacleDistance();
    printf("\U0001F6E1 Distance obstacle : %.2f m\n", min_distance);

    Real k = 2.0f;
    Real VmaxAvance = 100.0f;  
    Real vitesseAvance = VmaxAvance * (1 - exp(-k * (min_distance - 0.2)));  

    if (min_distance <= m_fStopDistance + 0.01) {
        printf("\u26A0️ OBSTACLE DÉTECTÉ !\n");

        if (!m_bAvoidingObstacle) {
            m_bAvoidingObstacle = true;
            m_bTurning = true;
            m_fThetaTarget = CRadians(m_fTheta) + CRadians::PI_OVER_TWO;
            printf("\U0001F3AF Nouveau cap cible (90° de rotation) : %.2f rad (%.2f°)\n", 
                   m_fThetaTarget.GetValue(), ToDegrees(m_fThetaTarget));
            
            m_pcProximity->Disable();
        }

        Real erreurCap = Sawtooth(CRadians(m_fThetaTarget) - CRadians(m_fTheta));
        Real kRotation = 0.5f;
        Real VmaxRotation = 10.0f; 

        Real Vd = -VmaxRotation * (1 - exp(-kRotation * erreurCap)); 
        Real Vg = VmaxRotation * (1 - exp(-kRotation * erreurCap));

        m_pcWheels->SetLinearVelocity(Vd, Vg);

        if (fabs(erreurCap) < 0.05) {  
            m_bTurning = false;
            m_bAvoidingObstacle = false;
            printf("\u2705 Rotation de 90° terminée, reprise de la marche.\n");

            m_pcProximity->Enable();
            m_pcWheels->SetLinearVelocity(vitesseAvance, vitesseAvance);
        }
    } else {
        m_pcWheels->SetLinearVelocity(vitesseAvance, vitesseAvance);
    }
}

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

