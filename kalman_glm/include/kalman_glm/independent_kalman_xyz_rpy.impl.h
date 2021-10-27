#pragma once

#include "kalman_glm/independent_kalman_xyz_rpy.h"

namespace kalman_glm {

    #define TEMPLATE_DEF template<class ValueType, class TimeType, class DurationType>
    #define CLASS_DECL IndependentKalmanXyzRpy<ValueType, TimeType, DurationType>

    TEMPLATE_DEF CLASS_DECL::~IndependentKalmanXyzRpy()
    {
    }
    
    TEMPLATE_DEF void CLASS_DECL::observe(time_type time, const glm::mat4& observation)
    {
        AbstractPoseRpyFilter::observe(time, observation);
    }
    TEMPLATE_DEF void CLASS_DECL::observe(time_type time, const PoseRpy& observation)
    {
        m_kalmanX.observe(time, glm::mat1x1(observation.position.x));
        m_kalmanY.observe(time, glm::mat1x1(observation.position.y));
        m_kalmanZ.observe(time, glm::mat1x1(observation.position.z));
        // todo: unwrap angles
        m_kalmanRoll.observe(time, glm::mat1x1(observation.roll_pitch_yaw.x));
        m_kalmanPitch.observe(time, glm::mat1x1(observation.roll_pitch_yaw.y));
        m_kalmanYaw.observe(time, glm::mat1x1(observation.roll_pitch_yaw.z));

        AbstractPoseRpyFilter::observe(time, observation);
    }

    TEMPLATE_DEF void CLASS_DECL::predict(time_type time)
    {
        m_kalmanX.predict(time);
        m_kalmanY.predict(time);
        m_kalmanZ.predict(time);
        m_kalmanRoll.predict(time);
        m_kalmanPitch.predict(time);
        m_kalmanYaw.predict(time);
    }

    TEMPLATE_DEF void CLASS_DECL::reset()
    {
        AbstractPoseRpyFilter::reset();
    }

    TEMPLATE_DEF void CLASS_DECL::set_filters_state(const PoseRpy& pose)
    {
        m_kalmanX.set_state(glm::mat1x1(pose.position.x));
        m_kalmanY.set_state(glm::mat1x1(pose.position.y));
        m_kalmanZ.set_state(glm::mat1x1(pose.position.z));
        m_kalmanRoll.set_state(glm::mat1x1(pose.roll_pitch_yaw.x));
        m_kalmanPitch.set_state(glm::mat1x1(pose.roll_pitch_yaw.y));
        m_kalmanYaw.set_state(glm::mat1x1(pose.roll_pitch_yaw.z));
    }
    
    TEMPLATE_DEF typename PoseRpy CLASS_DECL::get_filters_state()
    {
        typename PoseRpy pose;
        pose.position.x = m_kalmanX.state()[0][0];
        pose.position.y = m_kalmanY.state()[0][0];
        pose.position.z = m_kalmanZ.state()[0][0];
        pose.roll_pitch_yaw.x = m_kalmanRoll.state()[0][0];
        pose.roll_pitch_yaw.y = m_kalmanPitch.state()[0][0];
        pose.roll_pitch_yaw.z = m_kalmanYaw.state()[0][0];
        return pose;
    }

    TEMPLATE_DEF void CLASS_DECL::reset_filters() 
    {
        m_kalmanX.reset();
        m_kalmanY.reset();
        m_kalmanZ.reset();
        m_kalmanRoll.reset();
        m_kalmanPitch.reset();
        m_kalmanYaw.reset();

        m_kalmanX.set_state_uncertainty(glm::mat2x2(1e5));
        m_kalmanY.set_state_uncertainty(glm::mat2x2(1e5));
        m_kalmanZ.set_state_uncertainty(glm::mat2x2(1e5));
        m_kalmanRoll.set_state_uncertainty(glm::mat2x2(1e5));
        m_kalmanPitch.set_state_uncertainty(glm::mat2x2(1e5));
        m_kalmanYaw.set_state_uncertainty(glm::mat2x2(1e5));
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_invariant_position_process_uncertainty(const FilterProcessUncertainty& value)
    {
        m_time_invariant_position_process_uncertainty = value;
        m_kalmanX.m_time_invariant_process_uncertainty = value;
        m_kalmanY.m_time_invariant_process_uncertainty = value;
        m_kalmanZ.m_time_invariant_process_uncertainty = value;
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_variant_position_process_uncertainty(const FilterProcessUncertainty& value)
    {
        m_time_variant_position_process_uncertainty = value;
        m_kalmanX.m_time_variant_process_uncertainty = value;
        m_kalmanY.m_time_variant_process_uncertainty = value;
        m_kalmanZ.m_time_variant_process_uncertainty = value;
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_invariant_angle_process_uncertainty(const FilterProcessUncertainty& value)
    {
        m_time_invariant_angle_process_uncertainty = value;
        m_kalmanRoll.m_time_invariant_process_uncertainty = value;
        m_kalmanPitch.m_time_invariant_process_uncertainty = value;
        m_kalmanYaw.m_time_invariant_process_uncertainty = value;
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_variant_angle_process_uncertainty(const FilterProcessUncertainty& value)
    {
        m_time_variant_angle_process_uncertainty = value;
        m_kalmanRoll.m_time_variant_process_uncertainty = value;
        m_kalmanPitch.m_time_variant_process_uncertainty = value;
        m_kalmanYaw.m_time_variant_process_uncertainty = value;
    }


    TEMPLATE_DEF void CLASS_DECL::set_time_invariant_position_observation_uncertainty(const FilterObservationUncertainty& value)
    {
        m_time_invariant_position_observation_uncertainty = value;
        m_kalmanX.m_time_invariant_observation_uncertainty = value;
        m_kalmanY.m_time_invariant_observation_uncertainty = value;
        m_kalmanZ.m_time_invariant_observation_uncertainty = value;
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_variant_position_observation_uncertainty(const FilterObservationUncertainty& value)
    {
        m_time_variant_position_observation_uncertainty = value;
        m_kalmanX.m_time_variant_observation_uncertainty = value;
        m_kalmanY.m_time_variant_observation_uncertainty = value;
        m_kalmanZ.m_time_variant_observation_uncertainty = value;
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_invariant_angle_observation_uncertainty(const FilterObservationUncertainty& value)
    {
        m_time_invariant_angle_observation_uncertainty = value;
        m_kalmanRoll.m_time_invariant_observation_uncertainty = value;
        m_kalmanPitch.m_time_invariant_observation_uncertainty = value;
        m_kalmanYaw.m_time_invariant_observation_uncertainty = value;
    }

    TEMPLATE_DEF void CLASS_DECL::set_time_variant_angle_observation_uncertainty(const FilterObservationUncertainty& value)
    {
        m_time_variant_angle_observation_uncertainty = value;
        m_kalmanRoll.m_time_variant_observation_uncertainty = value;
        m_kalmanPitch.m_time_variant_observation_uncertainty = value;
        m_kalmanYaw.m_time_variant_observation_uncertainty = value;
    }


    #undef TEMPLATE_DEF
    #undef CLASS_DECL

} // namespace kalman_glm
