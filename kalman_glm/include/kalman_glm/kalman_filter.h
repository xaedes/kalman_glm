#pragma once
#include <glm/glm.hpp>
#include <type_traits>

#include "glm/ext/fwd_mat1.hpp"
#include "glm/ext/matrix_mat1.hpp"

namespace kalman_glm {

    template<
        int NumStates,
        int NumObservations,
        class ValueType = float
    >
    struct KalmanFilter
    {
        using num_states = std::integral_constant<int, NumStates>;
        using num_observations = std::integral_constant<int, NumObservations>;
        using value_type = ValueType;
        
        static_assert((1 <= num_states::value) && (num_states::value <= 4), "violated: 1 <= num_states <= 4");
        static_assert((1 <= num_observations::value) && (num_observations::value <= 4), "violated: 1 <= num_observations <= 4");

        using State                  = glm::mat<1               , NumStates       , value_type>;
        using StateTransitionMatrix  = glm::mat<NumStates       , NumStates       , value_type>;
        using StateUncertainty       = glm::mat<NumStates       , NumStates       , value_type>;
        using ProcessUncertainty     = glm::mat<NumStates       , NumStates       , value_type>;
        using ObservationMatrix      = glm::mat<NumStates       , NumObservations , value_type>;
        using ObservationUncertainty = glm::mat<NumObservations , NumObservations , value_type>;
        using Observation            = glm::mat<1               , NumObservations , value_type>;
        using KalmanGain             = glm::mat<NumStates       , NumObservations , value_type>;

        KalmanFilter(
            State state                                    = State(0),
            StateTransitionMatrix state_transition_matrix  = StateTransitionMatrix(1),
            StateUncertainty state_uncertainty             = StateUncertainty(1),
            ProcessUncertainty process_uncertainty         = ProcessUncertainty(1),
            ObservationMatrix observation_matrix           = ObservationMatrix(1),
            ObservationUncertainty observation_uncertainty = ObservationUncertainty(1)
        );
        virtual ~KalmanFilter();

        virtual void observe(const Observation& observation);
        virtual void predict();

        virtual void reset();

        bool has_observation() const { return m_has_observation; }

        const State&                  state()                   const { return m_x; }
        const StateTransitionMatrix&  state_transition_matrix() const { return m_F; }
        const StateUncertainty&       state_uncertainty()       const { return m_P; }
        const ProcessUncertainty&     process_uncertainty()     const { return m_Q; }
        const ObservationMatrix&      observation_matrix()      const { return m_H; }
        const ObservationUncertainty& observation_uncertainty() const { return m_R; }
        value_type                    regularization()          const { return m_regularization; }

        void set_state                   (const State&                  value) { m_x = value; }
        void set_state_transition_matrix (const StateTransitionMatrix&  value) { m_F = value; }
        void set_state_uncertainty       (const StateUncertainty&       value) { m_P = value; }
        void set_process_uncertainty     (const ProcessUncertainty&     value) { m_Q = value; }
        void set_observation_matrix      (const ObservationMatrix&      value) { m_H = value; }
        void set_observation_uncertainty (const ObservationUncertainty& value) { m_R = value; }
        void set_regularization(value_type& value) { m_regularization = value; }

        void condition_state_uncertainty();
    protected:

        State                  m_x; 
        StateTransitionMatrix  m_F; 
        StateUncertainty       m_P; 
        ProcessUncertainty     m_Q; 
        ObservationMatrix      m_H; 
        ObservationUncertainty m_R; 
        
        StateUncertainty       m_I;  // identity matrix
        bool m_has_observation = false; 
        value_type m_regularization = 1e-9;
    };

} // kalman_glm

#include "kalman_glm/kalman_filter.impl.h"
