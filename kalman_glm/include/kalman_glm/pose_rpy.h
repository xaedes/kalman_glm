#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>

namespace kalman_glm {

    struct PoseRpy
    {
        glm::vec3 position = glm::vec3(0,0,0);
        glm::vec3 roll_pitch_yaw = glm::vec3(0,0,0);

        static PoseRpy mat4_to_pose(const glm::mat4& mat)
        {
            PoseRpy pose;
            pose.position = mat[3];
            glm::extractEulerAngleXYZ(
                mat, 
                pose.roll_pitch_yaw.x, 
                pose.roll_pitch_yaw.y, 
                pose.roll_pitch_yaw.z
            );
            return pose;
        }
        
        static glm::mat4 pose_to_mat4(const PoseRpy& pose)
        {
            glm::mat4 mat = glm::eulerAngleXYZ(
                pose.roll_pitch_yaw.x,
                pose.roll_pitch_yaw.y,
                pose.roll_pitch_yaw.z
            );
            mat[3].xyz = pose.position;
            return mat;
        }
    };

} // namespace kalman_glm
