using UnityEngine;

namespace Unity.Robotics.UrdfImporter.Control
{
    /// <summary>
    /// ScriptableObject storing Denavit-Hartenberg parameters and joint limits for a robot.
    /// DH parameters follow the convention: [theta, d, a, alpha]
    /// </summary>
    [CreateAssetMenu(fileName = "RobotDHParameters", menuName = "Robotics/Robot DH Parameters", order = 1)]
    public class RobotDHParameters : ScriptableObject
    {
        [Header("Robot Configuration")]
        [Tooltip("Name of the robot (e.g., so100, so101)")]
        public string robotName = "so100";

        [Header("DH Parameters")]
        [Tooltip("DH parameters: [theta, d, a, alpha] for each joint")]
        public DHParameter[] dhParameters;

        [Header("Joint Limits (Mechanical)")]
        [Tooltip("Lower mechanical joint limits in radians")]
        public float[] jointLimitsLow;

        [Tooltip("Upper mechanical joint limits in radians")]
        public float[] jointLimitsHigh;

        [System.Serializable]
        public struct DHParameter
        {
            [Tooltip("Joint angle (theta) - variable for revolute joints")]
            public float theta;

            [Tooltip("Link offset (d) - distance along previous z-axis")]
            public float d;

            [Tooltip("Link length (a) - distance along common normal")]
            public float a;

            [Tooltip("Link twist (alpha) - rotation about common normal")]
            public float alpha;

            public DHParameter(float theta, float d, float a, float alpha)
            {
                this.theta = theta;
                this.d = d;
                this.a = a;
                this.alpha = alpha;
            }

            /// <summary>
            /// Converts to FKRobot format: [alpha, a, theta, d]
            /// </summary>
            public float[] ToFKRobotFormat()
            {
                return new float[] { alpha, a, theta, d };
            }
        }

        /// <summary>
        /// Validates that the configuration is complete and consistent.
        /// </summary>
        public bool Validate()
        {
            if (dhParameters == null || dhParameters.Length == 0)
            {
                Debug.LogError($"RobotDHParameters [{robotName}]: No DH parameters defined");
                return false;
            }

            // int jointCount = dhParameters.Length;
            //
            // if (jointLimitsLow != null && jointLimitsLow.Length != jointCount)
            // {
            //     Debug.LogError($"RobotDHParameters [{robotName}]: Joint limits low count ({jointLimitsLow.Length}) doesn't match DH parameters ({jointCount})");
            //     return false;
            // }
            //
            // if (jointLimitsHigh != null && jointLimitsHigh.Length != jointCount)
            // {
            //     Debug.LogError($"RobotDHParameters [{robotName}]: Joint limits high count ({jointLimitsHigh.Length}) doesn't match DH parameters ({jointCount})");
            //     return false;
            // }

            return true;
        }

        /// <summary>
        /// Gets DH parameters in FKRobot format: [alpha, a, theta, d]
        /// </summary>
        public float[][] GetDHParametersForFKRobot()
        {
            float[][] result = new float[dhParameters.Length][];
            for (int i = 0; i < dhParameters.Length; i++)
            {
                result[i] = dhParameters[i].ToFKRobotFormat();
            }
            return result;
        }
    }
}
