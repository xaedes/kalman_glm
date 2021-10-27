#include "kalman_glm/kalman_filter.h"

namespace kalman_glm {

    #define TEMPLATE_DEF template<int NumStates, int NumObservations, class ValueType>
    #define CLASS_DECL KalmanFilter<NumStates,NumObservations,ValueType>
    
    TEMPLATE_DEF CLASS_DECL::KalmanFilter()
    {
        m_x = State(0);
        m_F = StateTransitionMatrix(1);
        m_P = StateUncertainty(1);
        m_Q = ProcessUncertainty(1);
        m_H = ObservationMatrix(0);
        m_R = ObservationUncertainty(1);
        // identity
        m_I = StateUncertainty(1);
    }

    TEMPLATE_DEF CLASS_DECL::~KalmanFilter()
    {}

    TEMPLATE_DEF void CLASS_DECL::reset()
    {
        m_x *= 0;
        m_has_observation = false;
    }
    
    TEMPLATE_DEF void CLASS_DECL::condition_state_uncertainty()
    {
        set_state_uncertainty(
            // force symmetry
            0.5f * (state_uncertainty() + glm::transpose(state_uncertainty()))
            // force positive definiteness
            + regularization() * m_I
        );
    }

    TEMPLATE_DEF void CLASS_DECL::observe(const Observation& observation)
    {
        predict();

        // http://services.eng.uts.edu.au/~sdhuang/Extended%20Kalman%20Filter_Shoudong.pdf
        // http://de.wikipedia.org/wiki/Kalman-Filter#Korrektur
        
        auto innovation = (      // (1,NumObs)
            observation          // (1,NumObs)
          - observation_matrix() // (NumState,NumObs)
          * state()              // (1,NumState)
        );

        // auto observation_matrix_transpose = ObservationMatrix::transpose(observation_matrix());
        auto observation_matrix_transpose = glm::transpose(observation_matrix());

        ObservationUncertainty residual_covariance = (   // (NumObs,NumObs)
           (observation_matrix()                         // (NumState,NumObs)
          * state_transition_matrix())                   // (NumState,NumState)
          * observation_matrix_transpose                 // (NumObs,NumState)
          + observation_uncertainty()                    // (NumObs,NumObs)
        );

        auto inv_residual_covariance = glm::inverse(residual_covariance);

        auto kalman_gain = (             // (NumObs,NumState)
            state_uncertainty()          // (NumState,NumState)
          * observation_matrix_transpose // (NumObs,NumState)
          * inv_residual_covariance      // (NumObs,NumObs)
        );
        auto kalman_gain_transpose = glm::transpose(kalman_gain);

        set_state(      // (1,NumState)
            state()     // (1,NumState)
          + kalman_gain // (NumObs,NumState)
          * innovation  // (1,NumObs)
        );

        set_state_uncertainty(
            state_uncertainty()
          - kalman_gain
          * residual_covariance
          * kalman_gain_transpose
        );

        condition_state_uncertainty();

        m_has_observation = true;
    }

    TEMPLATE_DEF void CLASS_DECL::predict()
    {
        if (!has_observation()) return;
        set_state(
            state_transition_matrix() * state() 
        );

        auto state_transition_matrix_transpose = glm::transpose(state_transition_matrix());
        set_state_uncertainty(
            state_transition_matrix() 
          * state_uncertainty()
          * state_transition_matrix_transpose
          + process_uncertainty()
        );

        condition_state_uncertainty();
    }

    #undef TEMPLATE_DEF
    #undef CLASS_DECL
    
} // kalman_glm
