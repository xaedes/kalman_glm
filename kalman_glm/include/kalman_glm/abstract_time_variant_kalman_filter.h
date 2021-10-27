#pragma once

#include "kalman_glm/kalman_filter.h"

namespace kalman_glm {

    template<
        int NumStates,
        int NumObservations,
        class ValueType = float,
        class TimeType = double,
        class DurationType = float
    >
    struct AbstractTimeVariantKalmanFilter : public KalmanFilter<NumStates, NumObservations, ValueType>
    {
        using time_type = TimeType;
        using duration_type = DurationType;

        AbstractTimeVariantKalmanFilter();
        virtual ~AbstractTimeVariantKalmanFilter();

        virtual void observe(time_type time, const Observation& observation);
        virtual void predict(time_type time);

        virtual void reset();

        virtual void update_time_variant(time_type time, duration_type dt) = 0;

        time_type time() const { return m_time; }
        bool has_time() const { return m_has_time; }
        
        value_type min_dt() const { return m_min_dt; }
        void set_min_dt(value_type value) { return m_min_dt = value; }

    protected:
        time_type m_time;
        bool m_has_time = false;
        value_type m_min_dt = 1e-9;
    };

} // namespace kalman_glm

#include "kalman_glm/abstract_time_variant_kalman_filter.impl.h"
