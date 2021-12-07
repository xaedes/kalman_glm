#pragma once

#include "kalman_glm/abstract_pose_filter.h"

#include <glm/gtx/euler_angles.hpp>

namespace kalman_glm {

    #define TEMPLATE_DEF template<class PoseType, class ValueType, class TimeType, class DurationType>
    #define CLASS_DECL AbstractPoseFilter<PoseType, ValueType, TimeType, DurationType>

    TEMPLATE_DEF CLASS_DECL::AbstractPoseFilter()
    {}

    TEMPLATE_DEF CLASS_DECL::AbstractPoseFilter(const glm::mat4& transformation)
     : m_transformation(transformation)
    {
        m_transformation_inverse = glm::inverse(m_transformation);
    }

    TEMPLATE_DEF CLASS_DECL::~AbstractPoseFilter()
    {}

    TEMPLATE_DEF typename CLASS_DECL::pose_type CLASS_DECL::mat4_to_pose(const glm::mat4& mat) const
    {
        auto transformed = mat;
        if (enable_transpose()) transformed = glm::transpose(transformed);
        transformed = m_transformation * transformed;

        return pose_type::mat4_to_pose(transformed);
    }

    TEMPLATE_DEF glm::mat4 CLASS_DECL::pose_to_mat4(const pose_type& pose) const
    {
        glm::mat4 mat = pose_type::pose_to_mat4(pose);
        mat = m_transformation_inverse * mat;
        if (enable_transpose()) mat = glm::transpose(mat);
        return mat;
    }


    TEMPLATE_DEF void CLASS_DECL::observe(time_type time, const glm::mat4& observation)
    {
        auto pose = mat4_to_pose(observation);
        if (!has_observation())
        {
            set_filters_state(pose);
            m_has_observation = true;
        }
        this->observe(time, pose);
        m_state = get_filters_state();
    }

    TEMPLATE_DEF void CLASS_DECL::reset()
    {
        m_has_observation = false;

        reset_filters();
        
        m_state = get_filters_state();
    }

    TEMPLATE_DEF void CLASS_DECL::set_state(const pose_type& value) 
    { 
        m_state = value; 
        set_filters_state(m_state);
    }

    TEMPLATE_DEF void CLASS_DECL::set_transformation(const glm::mat4& value) 
    { 
        m_transformation = value; 
        m_transformation_inverse = glm::inverse(m_transformation); 
    }

    TEMPLATE_DEF void CLASS_DECL::set_enable_transpose(bool value)
    { 
        m_enable_transpose = value; 
    }

    #undef TEMPLATE_DEF
    #undef CLASS_DECL

} // namespace kalman_glm
