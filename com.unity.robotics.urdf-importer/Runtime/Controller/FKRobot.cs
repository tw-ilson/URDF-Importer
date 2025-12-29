using UnityEngine;
using System.Collections.Generic;
using System;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Unity.Robotics.UrdfImporter.Control{
    public class FKRobot : MonoBehaviour
    {
        private GameObject robot;
        private static int prismaticJointVariable = 3;
        private static int revoluteJointVariable = 2;
        public List<float[]> dh;
        public List<ArticulationBody> jointChain;
        public List<float> currentAngles;

        public Quaternion endEffectorRotation;
        public Matrix4x4 endEffectorPosition;

        public const string k_TagName = "robot";

        [Header("DH Parameter Configuration")]
        [Tooltip("ScriptableObject containing DH parameters")]
        public RobotDHParameters dhParametersAsset;

        [Header("FK Visualization")]
        public bool showFKGizmo = true;
        public GameObject visualizationSphere;
        public float gizmoSize = 0.05f;

        // TODO : Automatically adding DH parameters 2. Detecting number of active joints
        void Start()
        {
            if (dh == null)
            {
                dh = new List<float[]>();
            }
            
            jointChain = new List<ArticulationBody>();

            robot = FindRobotObject();
            
            if (!robot)
            {
                Debug.Log("Could not find robot!");
                return;
            }

            foreach (ArticulationBody joint in robot.GetComponentsInChildren<ArticulationBody>())
            {
                if (joint.jointType != ArticulationJointType.FixedJoint)
                    jointChain.Add(joint);
            }

            // Load from ScriptableObject if provided
            if (LoadDHParametersFromAsset())
            {
                Debug.Log("FKRobot: Loaded DH parameters from ScriptableObject");
            }
        }

        public static GameObject FindRobotObject()
        {
            try
            {
                GameObject robot = GameObject.FindWithTag(k_TagName);
                if (robot == null)
                {
                    Debug.LogWarning($"No GameObject with tag '{k_TagName}' was found.");
                }
                return robot;
            }
            catch (Exception)
            {
                Debug.LogError($"Unable to find tag '{k_TagName}'. " + 
                               $"Add A tag '{k_TagName}' in the Project Settings in Unity Editor.");
            }
            return null;
        }

        void FixedUpdate()
        {
            // Test angles: q_init = [-π/2, -π/2, π/2, π/2, -π/2, π/2]
            List<float> testAngles = new List<float>
            {
                -Mathf.PI / 2f,
                -Mathf.PI / 2f,
                 Mathf.PI / 2f,
                 Mathf.PI / 2f,
                -Mathf.PI / 2f,
                 Mathf.PI / 2f
            };

            FK(testAngles);
            UpdateVisualizationSphere();
        }

        /// <summary>
        /// Updates the visualization sphere position to match the FK-calculated end-effector position.
        /// Converts from FLU coordinate space to Unity coordinate space for proper visualization.
        /// </summary>
        private void UpdateVisualizationSphere()
        {
            if (visualizationSphere != null && endEffectorPosition != Matrix4x4.zero)
            {
                // Extract FK position in FLU coordinate space
                Vector3 fkPositionFLU = new Vector3(
                    endEffectorPosition.m03,
                    endEffectorPosition.m13,
                    endEffectorPosition.m23
                );

                // Convert from FLU to Unity coordinates
                Vector3 fkPosition = FLUToUnity(fkPositionFLU);

                // Position the visualization sphere at the FK-calculated end-effector position
                visualizationSphere.transform.position = fkPosition;

                // Optionally, also update rotation to match FK-calculated orientation
                Vector3 xAxisFLU = new Vector3(endEffectorPosition.m00, endEffectorPosition.m10, endEffectorPosition.m20);
                Vector3 yAxisFLU = new Vector3(endEffectorPosition.m01, endEffectorPosition.m11, endEffectorPosition.m21);
                Vector3 zAxisFLU = new Vector3(endEffectorPosition.m02, endEffectorPosition.m12, endEffectorPosition.m22);

                Vector3 xAxis = FLUToUnity(xAxisFLU);
                Vector3 yAxis = FLUToUnity(yAxisFLU);
                Vector3 zAxis = FLUToUnity(zAxisFLU);

                // Construct rotation matrix in Unity space
                Matrix4x4 rotationMatrix = new Matrix4x4();
                rotationMatrix.SetColumn(0, new Vector4(xAxis.x, xAxis.y, xAxis.z, 0));
                rotationMatrix.SetColumn(1, new Vector4(yAxis.x, yAxis.y, yAxis.z, 0));
                rotationMatrix.SetColumn(2, new Vector4(zAxis.x, zAxis.y, zAxis.z, 0));
                rotationMatrix.SetColumn(3, new Vector4(0, 0, 0, 1));

                visualizationSphere.transform.rotation = rotationMatrix.rotation;
            }
        }


        /// <summary>
        /// Returns a homogenous transform which moves the end effector co-ordinate system to the world co-ordinate system
        /// </summary>
        /// <param name="angles">List of float numbers representing joint poistions of a robot in meters or radians.</param>
        /// <returns></returns>
        public Matrix4x4 FK(List<float> angles = null)
        {
            currentAngles = currentJointParameters();

            List<float> jointAngles;
            if (angles == null)
            {
                jointAngles = currentAngles;
            }
            else
            {
                jointAngles = angles;
            }

            // Use first articulation body as base position for FK calculations
            ArticulationBody baseJoint = jointChain[0];

            Matrix4x4 baseTransform = GetBaseFLUTransform(baseJoint.transform);

            endEffectorPosition = baseTransform;

            int count = Math.Min(dh.Count, jointAngles.Count);
            for (int i = 0; i < count; i++)
            {
                float jointAngle = jointAngles[i];
                endEffectorPosition = endEffectorPosition * FWDMatrix2(dh[i], jointAngle);
            }

            return endEffectorPosition;

        }

        /// <summary>
        /// Creates a copy of the DH parameters with current joint positions added.
        /// For revolute joints: Theta_total = Theta_dh + Theta_joint
        /// For prismatic joints: d_total = d_dh + d_joint
        /// JointPostition: 0-2: rotation along XYZ axis 3-5: Translation along XYZ co-ordinates
        /// https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody-jointPosition.html
        /// </summary>
        /// <param name="angles">List of float numbers representing joint poistions of a robot in meters or radians</param>
        /// <returns>Deep copy of DH parameters with joint values added</returns>
        public List<float[]> PopulateDHparameters(List<float> angles)
        {
            // Create a deep copy of the DH parameters
            List<float[]> dhCopy = new List<float[]>();
            for (int i = 0; i < dh.Count; i++)
            {
                dhCopy.Add((float[])dh[i].Clone());
            }

            // Iterate over DH parameters, updating with joint values where available
            for (int i = 0; i < dh.Count; i++)
            {
                if (i >= jointChain.Count)
                    break;

                if (jointChain[i].jointType == ArticulationJointType.RevoluteJoint)
                {
                    // Add joint position to existing theta parameter
                    dhCopy[i][revoluteJointVariable] += jointChain[i].jointPosition[0];
                }
                else if (jointChain[i].jointType == ArticulationJointType.PrismaticJoint)
                {
                    // Add joint position to existing d parameter
                    dhCopy[i][prismaticJointVariable] += jointChain[i].jointPosition[3];
                }
                else
                {
                    Debug.LogError("Other joint types not supported");
                }
            }

            return dhCopy;
        }

        public List<float> currentJointParameters()
        {
            List<float> angles = new List<float>();
            for (int i = 0; i < jointChain.Count; i++)
            {
                angles.Add((float)Math.Round(jointChain[i].jointPosition[0],2));
            }
            return angles;
        }

        /// <summary>
        /// Returns a homogenous transformatino matrix formed using a set of DH paramters of a joint in new DH convention.
        /// https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters
        /// https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody-jointPosition.html
        /// </summary>
        /// <param name="DHparameters">Array of four float parameters</param>
        /// <returns></returns>
        private Matrix4x4 FWDMatrix(float[] DHparameters) 
        {
            return new Matrix4x4(new Vector4(Mathf.Cos(DHparameters[2]),-Mathf.Sin(DHparameters[2]),0,DHparameters[1]),
                                new Vector4(Mathf.Sin(DHparameters[2]) * Mathf.Cos(DHparameters[0]),Mathf.Cos(DHparameters[2]) * Mathf.Cos(DHparameters[0]),-Mathf.Sin(DHparameters[0]),-Mathf.Sin(DHparameters[0]) * DHparameters[3]),
                                new Vector4(Mathf.Sin(DHparameters[2]) * Mathf.Sin(DHparameters[0]),Mathf.Cos(DHparameters[2]) * Mathf.Sin(DHparameters[0]), Mathf.Cos(DHparameters[0]),Mathf.Cos(DHparameters[0]) * DHparameters[3]),
                                new Vector4(0,0,0,1));
        }

        /// <summary>
        /// Returns a homogenous transformation matrix formed using a set of DH parameters of a joint in modified DH convention.
        /// The joint angle is added to the theta parameter (DHparameters[2]) for revolute joints.
        /// https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters
        /// </summary>
        /// <param name="DHparameters">Array of four float parameters [alpha, a, theta, d]</param>
        /// <param name="jointAngle">Joint angle to add to theta (for revolute joints) or d (for prismatic joints)</param>
        /// <returns>Transformation matrix</returns>
        public Matrix4x4 FWDMatrix2(float[] DHparameters, float jointAngle)
        {
            float theta = DHparameters[2] + jointAngle;
            float alpha = DHparameters[0];
            float a = DHparameters[1];
            float d = DHparameters[3];

            float cosTheta = Mathf.Cos(theta);
            float sinTheta = Mathf.Sin(theta);
            float cosAlpha = Mathf.Cos(alpha);
            float sinAlpha = Mathf.Sin(alpha);

            Matrix4x4 T = new Matrix4x4(
                new Vector4(cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta),
                new Vector4(sinTheta, cosTheta * cosAlpha, -sinAlpha * cosTheta, sinTheta * a),
                new Vector4(0, sinAlpha, cosAlpha, d),
                new Vector4(0, 0, 0, 1)
            );

            return T.transpose;
        }

        /// <summary>
        /// Converts a vector from FLU coordinate space to Unity coordinate space.
        /// FLU: X=forward, Y=left, Z=up -> Unity: X=right, Y=up, Z=forward
        /// </summary>
        private Vector3 FLUToUnity(Vector3 fluVector)
        {
            // Unity.X = -FLU.Y, Unity.Y = FLU.Z, Unity.Z = FLU.X
            return new Vector3(-fluVector.y, fluVector.z, fluVector.x);
        }

        /// <summary>
        /// Converts a vector from Unity coordinate space to FLU coordinate space.
        /// Unity: X=right, Y=up, Z=forward -> FLU: X=forward, Y=left, Z=up
        /// </summary>
        private Vector3 UnityToFLU(Vector3 unityVector)
        {
            // FLU.X = Unity.Z, FLU.Y = -Unity.X, FLU.Z = Unity.Y
            return new Vector3(unityVector.z, -unityVector.x, unityVector.y);
        }

        /// <summary>copy
        /// Creates the base frame transformation matrix in FLU coordinate space.
        /// Converts the base joint position from Unity to FLU and creates a translation matrix.
        /// </summary>
        /// <param name="baseJointTransform">The transform of the base joint</param>
        /// <returns>Base transformation matrix in FLU space</returns>
        private Matrix4x4 GetBaseFLUTransform(Transform baseJointTransform)
        {
            // Convert base position from Unity to FLU coordinates
            Vector3 basePosFLU = UnityToFLU(baseJointTransform.position);

            // Create translation matrix in FLU space
            return Matrix4x4.Translate(basePosFLU);
        }

        private void OnDrawGizmos()
        {
            if (showFKGizmo && robot != null && jointChain != null && jointChain.Count > 0)
            {
                // Draw FLU origin coordinate frame
                Vector3 fluOrigin = FLUToUnity(Vector3.zero);
                Vector3 fluOriginXAxis = FLUToUnity(new Vector3(1, 0, 0)) * gizmoSize * 3;
                Vector3 fluOriginYAxis = FLUToUnity(new Vector3(0, 1, 0)) * gizmoSize * 3;
                Vector3 fluOriginZAxis = FLUToUnity(new Vector3(0, 0, 1)) * gizmoSize * 3;

                Gizmos.color = Color.red;
                Gizmos.DrawLine(fluOrigin, fluOrigin + fluOriginXAxis);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(fluOrigin, fluOrigin + fluOriginYAxis);
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(fluOrigin, fluOrigin + fluOriginZAxis);

#if UNITY_EDITOR
                Handles.Label(fluOrigin + Vector3.up * gizmoSize * 0.5f, "Origin", new GUIStyle {
                    normal = new GUIStyleState { textColor = Color.white },
                    fontSize = 12,
                    fontStyle = FontStyle.Bold
                });
#endif

                // Use first articulation body as base position for FK calculations
                ArticulationBody baseJoint = jointChain[0];

                Matrix4x4 baseTransform = GetBaseFLUTransform(baseJoint.transform);
                Matrix4x4 accumulatedTransformFLU = baseTransform;


                // Extract position in FLU coordinate space and convert to Unity
                Vector3 basePositionFLU = new Vector3(
                    accumulatedTransformFLU.m03,
                    accumulatedTransformFLU.m13,
                    accumulatedTransformFLU.m23
                );

                Vector3 basePosition = FLUToUnity(basePositionFLU);

#if UNITY_EDITOR
                // Display debug info as labels in scene view
                string debugInfo = $"Base Joint Unity Pos: {baseJoint.transform.position}\n" +
                                   $"Base Pos FLU: {basePositionFLU}\n" +
                                   $"Base Pos (Unity): {basePosition}\n" +
                                   $"BaseTransform m03,m13,m23: ({baseTransform.m03}, {baseTransform.m13}, {baseTransform.m23})";

                Handles.Label(Vector3.zero + Vector3.up * 2f, debugInfo, new GUIStyle {
                    normal = new GUIStyleState { textColor = Color.white },
                    fontSize = 11,
                    alignment = TextAnchor.UpperLeft
                });
#endif

                // Extract rotation columns from the transformation matrix (in FLU space)
                Vector3 baseXAxisFLU = new Vector3(accumulatedTransformFLU.m00, accumulatedTransformFLU.m10, accumulatedTransformFLU.m20);
                Vector3 baseYAxisFLU = new Vector3(accumulatedTransformFLU.m01, accumulatedTransformFLU.m11, accumulatedTransformFLU.m21);
                Vector3 baseZAxisFLU = new Vector3(accumulatedTransformFLU.m02, accumulatedTransformFLU.m12, accumulatedTransformFLU.m22);

                // Convert axes from FLU to Unity
                Vector3 baseXAxis = FLUToUnity(baseXAxisFLU);
                Vector3 baseYAxis = FLUToUnity(baseYAxisFLU);
                Vector3 baseZAxis = FLUToUnity(baseZAxisFLU);

                // Draw sphere at base frame origin
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(basePosition, gizmoSize);

                Gizmos.color = Color.red;
                Gizmos.DrawLine(basePosition, basePosition + baseXAxis * gizmoSize * 1.5f);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(basePosition, basePosition + baseYAxis * gizmoSize * 1.5f);
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(basePosition, basePosition + baseZAxis * gizmoSize * 1.5f);

#if UNITY_EDITOR
                Handles.Label(basePosition + Vector3.up * gizmoSize * 0.3f, "Base", new GUIStyle {
                    normal = new GUIStyleState { textColor = Color.cyan },
                    fontSize = 12,
                    fontStyle = FontStyle.Bold
                });
#endif

                // Draw intermediate frames for each joint
                if (dh != null && dh.Count > 0)
                {
                    // Perform FK calculations in FLU (ROS) coordinate space
                    // Start with base frame transformation
                    List<float> jointAngles = currentJointParameters();

                    int count = Math.Min(dh.Count, jointAngles.Count);
                    for (int i = 0; i < count; i++)
                    {
                        // Compute FK up to joint i in FLU space
                        float jointAngle = jointAngles[i];
                        accumulatedTransformFLU = accumulatedTransformFLU * FWDMatrix2(dh[i], jointAngle);

                        // Extract position in FLU coordinate space
                        Vector3 framePositionFLU = new Vector3(
                            accumulatedTransformFLU.m03,
                            accumulatedTransformFLU.m13,
                            accumulatedTransformFLU.m23
                        );

                        // Convert from FLU to Unity coordinates
                        Vector3 framePosition = FLUToUnity(framePositionFLU);

                        // Extract rotation columns from the transformation matrix
                        // GetColumn extracts the rotated basis vectors
                        Vector3 xAxisFLU = new Vector3(accumulatedTransformFLU.m00, accumulatedTransformFLU.m10, accumulatedTransformFLU.m20);
                        Vector3 yAxisFLU = new Vector3(accumulatedTransformFLU.m01, accumulatedTransformFLU.m11, accumulatedTransformFLU.m21);
                        Vector3 zAxisFLU = new Vector3(accumulatedTransformFLU.m02, accumulatedTransformFLU.m12, accumulatedTransformFLU.m22);

                        Vector3 xAxis = FLUToUnity(xAxisFLU);
                        Vector3 yAxis = FLUToUnity(yAxisFLU);
                        Vector3 zAxis = FLUToUnity(zAxisFLU);

                        // Draw coordinate axes
                        Gizmos.color = Color.red;
                        Gizmos.DrawLine(framePosition, framePosition + xAxis * gizmoSize);
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(framePosition, framePosition + yAxis * gizmoSize);
                        Gizmos.color = Color.blue;
                        Gizmos.DrawLine(framePosition, framePosition + zAxis * gizmoSize);

#if UNITY_EDITOR
                        // Draw frame number label
                        Handles.Label(framePosition + Vector3.up * gizmoSize * 0.3f, $"Frame {i + 1}", new GUIStyle {
                            normal = new GUIStyleState { textColor = Color.white },
                            fontSize = 10
                        });
#endif
                    }

                    // Draw end-effector frame using accumulated transform (consistent with intermediate frames)
                    if (count > 0)
                    {
                        // FK position in FLU coordinate space (use accumulated transform)
                        Vector3 fkPositionFLU = new Vector3(
                            accumulatedTransformFLU.m03,
                            accumulatedTransformFLU.m13,
                            accumulatedTransformFLU.m23
                        );

                        // Convert from FLU to Unity coordinates
                        Vector3 fkPosition = FLUToUnity(fkPositionFLU);

                        // Draw sphere at FK end effector position
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawWireSphere(fkPosition, gizmoSize);

                        // Extract and convert orientation axes from FLU to Unity
                        Vector3 xAxisFLU = new Vector3(accumulatedTransformFLU.m00, accumulatedTransformFLU.m10, accumulatedTransformFLU.m20);
                        Vector3 yAxisFLU = new Vector3(accumulatedTransformFLU.m01, accumulatedTransformFLU.m11, accumulatedTransformFLU.m21);
                        Vector3 zAxisFLU = new Vector3(accumulatedTransformFLU.m02, accumulatedTransformFLU.m12, accumulatedTransformFLU.m22);

                        Vector3 xAxis = FLUToUnity(xAxisFLU);
                        Vector3 yAxis = FLUToUnity(yAxisFLU);
                        Vector3 zAxis = FLUToUnity(zAxisFLU);

                        Gizmos.color = Color.red;
                        Gizmos.DrawLine(fkPosition, fkPosition + xAxis * gizmoSize * 2);
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(fkPosition, fkPosition + yAxis * gizmoSize * 2);
                        Gizmos.color = Color.blue;
                        Gizmos.DrawLine(fkPosition, fkPosition + zAxis * gizmoSize * 2);

#if UNITY_EDITOR
                        Handles.Label(fkPosition + Vector3.up * gizmoSize * 0.3f, "End-Effector", new GUIStyle {
                            normal = new GUIStyleState { textColor = Color.yellow },
                            fontSize = 12,
                            fontStyle = FontStyle.Bold
                        });
#endif
                    }
                }
            }
        }

        /// <summary>
        /// Loads DH parameters from the assigned RobotDHParameters ScriptableObject asset.
        /// </summary>
        private bool LoadDHParametersFromAsset()
        {
            if (dhParametersAsset == null) {
                return false;
            }

            if (!dhParametersAsset.Validate())
            {
                Debug.LogError("[LoadDHParametersFromAsset] DH parameters asset validation failed - check the asset configuration");
                return false;
            }

            // Initialize dh list
            dh = new List<float[]>();

            // Load DH parameters in FKRobot format: [alpha, a, theta, d]
            foreach (var dhParam in dhParametersAsset.dhParameters)
            {
                dh.Add(dhParam.ToFKRobotFormat());
            }

            Debug.Log($"[LoadDHParametersFromAsset] SUCCESS - Loaded {dh.Count} DH parameters from {dhParametersAsset.name} ({dhParametersAsset.robotName})");
            LogDHParameters();

            return true;
        }


        /// <summary>
        /// Validates that computed DH parameters contain no invalid values.
        /// </summary>
        private bool ValidateDHParameters()
        {
            if (dh == null || dh.Count == 0)
            {
                Debug.LogError("FKRobot: DH parameters list is null or empty");
                return false;
            }

            for (int i = 0; i < dh.Count; i++)
            {
                if (dh[i] == null || dh[i].Length != 4)
                {
                    Debug.LogError($"FKRobot: DH parameter array at joint {i} is invalid");
                    return false;
                }

                for (int j = 0; j < 4; j++)
                {
                    if (float.IsNaN(dh[i][j]) || float.IsInfinity(dh[i][j]))
                    {
                        Debug.LogError($"FKRobot: DH parameters contain invalid value at joint {i}, parameter {j}");
                        return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Logs computed DH parameters for debugging and validation.
        /// </summary>
        private void LogDHParameters()
        {
            Debug.Log("FKRobot DH Parameters (Modified DH Convention):");
            Debug.Log("Joint | Name | α (rad) | a (m) | θ (rad) | d (m)");
            Debug.Log("------|------|---------|-------|---------|------");

            for (int i = 0; i < dh.Count; i++)
            {
                string jointName = jointChain[i] != null ? jointChain[i].name : "Unknown";
                Debug.Log($"  {i}   | {jointName} | {dh[i][0]:F4} | {dh[i][1]:F4} | {dh[i][2]:F4} | {dh[i][3]:F4}");
            }
        }

    }
}
