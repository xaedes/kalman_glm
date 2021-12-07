#pragma once

#include <glm/glm.hpp>

namespace kalman_glm {

    template<
        class PoseType,
        class ValueType = float,
        class TimeType = double,
        class DurationType = float
    >
    struct AbstractPoseFilter 
    {
        using pose_type = PoseType;
        using value_type = ValueType;
        using time_type = TimeType;
        using duration_type = DurationType;


        AbstractPoseFilter();
        AbstractPoseFilter(const glm::mat4& transformation);
        virtual ~AbstractPoseFilter();

        bool has_observation() const { return m_has_observation; }

        virtual void reset();
        virtual void observe(time_type time, const glm::mat4& observation);

        virtual void observe(time_type time, const pose_type& observation) = 0;
        virtual void predict(time_type time) = 0;


        pose_type  mat4_to_pose(const glm::mat4& mat) const;
        glm::mat4 pose_to_mat4(const pose_type& pose ) const;

        glm::mat4        pose_as_mat4()           const { return pose_to_mat4(state());    }
        const pose_type&  state()                  const { return m_state;                  }
        const glm::mat4& transformation()         const { return m_transformation;         }
        const glm::mat4& transformation_inverse() const { return m_transformation_inverse; }
        bool             enable_transpose()       const { return m_enable_transpose;       }
        
        void set_state(const pose_type& value);
        void set_transformation(const glm::mat4& value);
        void set_enable_transpose(bool value);

    protected:
        virtual void set_filters_state(const pose_type& value) = 0;
        virtual pose_type get_filters_state() = 0;
        virtual void reset_filters() = 0;

        pose_type m_state;

        bool m_has_observation = false;

        bool m_enable_transpose = false;
        glm::mat4 m_transformation = glm::mat4(1);
        glm::mat4 m_transformation_inverse = glm::mat4(1);



    };

} // namespace kalman_glm

#include "kalman_glm/abstract_pose_filter.impl.h"
