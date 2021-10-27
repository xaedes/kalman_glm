#pragma once

#include "kalman_glm/abstract_time_variant_kalman_filter.h"

namespace kalman_glm {

    #define TEMPLATE_DEF template<int NumStates, int NumObservations, class ValueType, class TimeType, class DurationType>
    #define CLASS_NAME AbstractTimeVariantKalmanFilter<NumStates,NumObservations,ValueType,TimeType,DurationType>

    TEMPLATE_DEF CLASS_NAME::AbstractTimeVariantKalmanFilter()
        : KalmanFilter<NumStates,NumObservations,ValueType>()
    {}

    TEMPLATE_DEF CLASS_NAME::~AbstractTimeVariantKalmanFilter()
    {}

    TEMPLATE_DEF void CLASS_NAME::observe(time_type time, const Observation& observation)
    {
        if (has_time())
        {
            auto dt = static_cast<duration_type>(time - m_time);
            update_time_variant(time, dt);
        }
        else
        {
            update_time_variant(time);
        }
        KalmanFilter<NumStates, NumObservations, ValueType>::observe(observation);
        m_time = time;
        m_has_time = true;
    }

    TEMPLATE_DEF void CLASS_NAME::predict(time_type time)
    {
        if (has_time())
        {
            auto dt = static_cast<duration_type>(time - m_time);
            update_time_variant(time, dt);
        }
        else
        {
            update_time_variant(time);
        }
        KalmanFilter<NumStates, NumObservations, ValueType>::predict();
        m_time = time;
        m_has_time = true;
    }

    TEMPLATE_DEF void CLASS_NAME::reset()
    {
        KalmanFilter<NumStates, NumObservations, ValueType>::reset();
        m_has_time = false;
    }

    #undef TEMPLATE_DEF

} // namespace kalman_glm
