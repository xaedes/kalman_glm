#pragma once

#include <glm/glm.hpp>
#include "kalman_glm/pose_rpy.h"

namespace kalman_glm {

    template<
        class ValueType = float,
        class TimeType = double,
        class DurationType = float
    >
    struct AbstractPoseRpyFilter 
    {
        using value_type = ValueType;
        using time_type = TimeType;
        using duration_type = DurationType;


        AbstractPoseRpyFilter();
        // transformation is applied to observations before filtering.
        // chose a transformation that will result in minimal angles for your use case.
        // avoid transfomations where some abs(angle) is close to pi/2.
        AbstractPoseRpyFilter(const glm::mat4& transformation);
        virtual ~AbstractPoseRpyFilter();

        bool has_observation() const { return m_has_observation; }

        virtual void reset();
        virtual void observe(time_type time, const glm::mat4& observation);

        virtual void observe(time_type time, const PoseRpy& observation);
        virtual void predict(time_type time) = 0;


        PoseRpy   mat4_to_pose(const glm::mat4& mat) const;
        glm::mat4 pose_to_mat4(const PoseRpy& pose ) const;

        glm::mat4        pose_as_mat4()           const { return pose_to_mat4(state());    }
        const PoseRpy&   state()                  const { return m_state;                  }
        const glm::mat4& transformation()         const { return m_transformation;         }
        const glm::mat4& transformation_inverse() const { return m_transformation_inverse; }
        bool             enable_transpose()       const { return m_enable_transpose;       }
        
        void set_state(const PoseRpy& value);
        void set_transformation(const glm::mat4& value);
        void set_enable_transpose(bool value);

    protected:
        virtual void set_filters_state(const PoseRpy& value) = 0;
        virtual PoseRpy get_filters_state() = 0;
        virtual void reset_filters() = 0;

        PoseRpy m_state;

        bool m_has_observation = false;

        bool m_enable_transpose = false;
        glm::mat4 m_transformation = glm::mat4(1);
        glm::mat4 m_transformation_inverse = glm::mat4(1);



    };

} // namespace kalman_glm

#include "kalman_glm/abstract_pose_rpy_filter.impl.h"
