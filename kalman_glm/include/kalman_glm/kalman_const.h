#pragma once

#include "kalman_glm/abstract_time_variant_kalman_filter.h"

namespace kalman_glm {

    template<
        class ValueType = float,
        class TimeType = double,
        class DurationType = float
    >
    struct KalmanConst : public AbstractTimeVariantKalmanFilter<1, 1, ValueType, TimeType, DurationType>
    {
        virtual ~KalmanConst() {}
        KalmanConst()
        {
            m_time_invariant_process_uncertainty     = ProcessUncertainty(0);
            m_time_variant_process_uncertainty       = ProcessUncertainty(1);
            m_time_invariant_observation_uncertainty = ObservationUncertainty(0);
            m_time_variant_observation_uncertainty   = ObservationUncertainty(1);
            set_observation_matrix(ObservationMatrix(1));
            set_state_transition_matrix(StateTransitionMatrix(1));
            
        }

        virtual void update_time_variant(time_type time, duration_type dt) override
        {
            set_observation_uncertainty(
                m_time_invariant_observation_uncertainty
              + dt * m_time_variant_observation_uncertainty
            );
            set_process_uncertainty(
                m_time_invariant_process_uncertainty 
              + dt * m_time_variant_process_uncertainty
            );
        }

        ProcessUncertainty m_time_invariant_process_uncertainty;
        ProcessUncertainty m_time_variant_process_uncertainty;

        ObservationUncertainty m_time_invariant_observation_uncertainty;
        ObservationUncertainty m_time_variant_observation_uncertainty;

    };

} // namespace kalman_glm
