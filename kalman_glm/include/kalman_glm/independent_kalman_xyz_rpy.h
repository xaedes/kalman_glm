#pragma once

#include <glm/glm.hpp>

#include "kalman_glm/abstract_pose_filter.h"
#include "kalman_glm/pose_rpy.h"
#include "kalman_glm/kalman_const.h"
#include "kalman_glm/kalman_kinematic_first_order.h"

namespace kalman_glm {

    template<
        class ValueType = float,
        class TimeType = double,
        class DurationType = float
    >
    struct IndependentKalmanXyzRpy : public AbstractPoseFilter<PoseRpy, ValueType, TimeType, DurationType>
    {
        using AbstractPoseFilter::AbstractPoseFilter;

        virtual ~IndependentKalmanXyzRpy();

        virtual void observe(time_type time, const glm::mat4& observation) override;
        virtual void observe(time_type time, const PoseRpy& observation) override;
        virtual void predict(time_type time) override;
        virtual void reset() override;

        // using Filter = KalmanConst<value_type, time_type, duration_type>;
        using Filter = KalmanKinematicFirstOrder<value_type, time_type, duration_type>;
        using FilterObservationUncertainty = typename Filter::ObservationUncertainty;
        using FilterProcessUncertainty = typename Filter::ProcessUncertainty;

        const FilterObservationUncertainty& time_invariant_angle_observation_uncertainty()    const { return m_time_invariant_angle_observation_uncertainty; }
        const FilterObservationUncertainty& time_invariant_position_observation_uncertainty() const { return m_time_invariant_position_observation_uncertainty; }
        const FilterObservationUncertainty& time_variant_angle_observation_uncertainty()      const { return m_time_variant_angle_observation_uncertainty; }
        const FilterObservationUncertainty& time_variant_position_observation_uncertainty()   const { return m_time_variant_position_observation_uncertainty; }

        const FilterProcessUncertainty& time_invariant_angle_process_uncertainty()            const { return m_time_invariant_angle_process_uncertainty; }
        const FilterProcessUncertainty& time_invariant_position_process_uncertainty()         const { return m_time_invariant_position_process_uncertainty; }
        const FilterProcessUncertainty& time_variant_angle_process_uncertainty()              const { return m_time_variant_angle_process_uncertainty; }
        const FilterProcessUncertainty& time_variant_position_process_uncertainty()           const { return m_time_variant_position_process_uncertainty; }


        void set_time_invariant_angle_observation_uncertainty   (const FilterObservationUncertainty& value);
        void set_time_invariant_angle_process_uncertainty       (const FilterProcessUncertainty& value);
        void set_time_invariant_position_observation_uncertainty(const FilterObservationUncertainty& value);
        void set_time_invariant_position_process_uncertainty    (const FilterProcessUncertainty& value);

        void set_time_variant_angle_observation_uncertainty     (const FilterObservationUncertainty& value);
        void set_time_variant_angle_process_uncertainty         (const FilterProcessUncertainty& value);
        void set_time_variant_position_observation_uncertainty  (const FilterObservationUncertainty& value);
        void set_time_variant_position_process_uncertainty      (const FilterProcessUncertainty& value);
    protected:

        virtual void set_filters_state(const PoseRpy& value) override;
        virtual PoseRpy get_filters_state() override;
        virtual void reset_filters() override;

        FilterObservationUncertainty m_time_invariant_angle_observation_uncertainty    = FilterObservationUncertainty(0);
        FilterObservationUncertainty m_time_invariant_position_observation_uncertainty = FilterObservationUncertainty(0);
        FilterObservationUncertainty m_time_variant_angle_observation_uncertainty      = FilterObservationUncertainty(1);
        FilterObservationUncertainty m_time_variant_position_observation_uncertainty   = FilterObservationUncertainty(1);

        FilterProcessUncertainty m_time_invariant_angle_process_uncertainty            = FilterProcessUncertainty(0);
        FilterProcessUncertainty m_time_invariant_position_process_uncertainty         = FilterProcessUncertainty(0);
        FilterProcessUncertainty m_time_variant_angle_process_uncertainty              = FilterProcessUncertainty(1);
        FilterProcessUncertainty m_time_variant_position_process_uncertainty           = FilterProcessUncertainty(1);

        Filter m_kalmanX;
        Filter m_kalmanY;
        Filter m_kalmanZ;
        Filter m_kalmanRoll;
        Filter m_kalmanPitch;
        Filter m_kalmanYaw;


    };

} // namespace kalman_glm

#include "kalman_glm/independent_kalman_xyz_rpy.impl.h"
