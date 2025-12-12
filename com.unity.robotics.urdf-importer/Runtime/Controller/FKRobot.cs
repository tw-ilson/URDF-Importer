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
        public bool autoComputeDH = false;

        [Tooltip("Optional: ScriptableObject containing pre-defined DH parameters")]
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

            // Priority 1: Load from ScriptableObject if provided
            if (LoadDHParametersFromAsset())
            {
                Debug.Log("FKRobot: Loaded DH parameters from ScriptableObject");
            }             
            // Priority 2: Auto-compute if enabled and no parameters set
            else if (autoComputeDH && (dh == null || dh.Count == 0))
            {
                AutoPopulateDHParameters();
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
            if (dh.Count == jointChain.Count)
            {
                FK();
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
            if (angles == null)
            {
                PopulateDHparameters(currentAngles);
            }
            else
            {
                PopulateDHparameters(angles);
            }
            endEffectorPosition = Matrix4x4.identity;
            for (int i = 0; i < dh.Count; i++)
            {
                Matrix4x4 temp = FWDMatrix2(dh[i]).transpose;
                endEffectorPosition = endEffectorPosition * (FWDMatrix2(dh[i]).transpose);
            }

            return endEffectorPosition;
        
        }

        /// <summary>
        /// Modifies the DH parameters with the current joint positions from the robot in Unity Simulation
        /// JointPostition: 0-2: rotation along XYZ axis 3-5: Translation along XYZ co-ordinates
        /// https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody-jointPosition.html
        /// </summary>
        /// <param name="angles">List of float numbers representing joint poistions of a robot in meters or radians</param>
        public void PopulateDHparameters(List<float> angles)
        {
            for (int i = 0; i < jointChain.Count; i++)
            {
                if (jointChain[i].jointType == ArticulationJointType.RevoluteJoint)
                {
                    dh[i][revoluteJointVariable] = jointChain[i].jointPosition[0];
                }
                else if (jointChain[i].jointType == ArticulationJointType.PrismaticJoint)
                {
                    dh[i][prismaticJointVariable] = jointChain[i].jointPosition[3];
                }
                else
                {
                    Debug.LogError("Other joint types not supported");
                }
            }


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
        /// Returns a homogenous transformatino matrix formed using a set of DH paramters of a joint in new DH convention.
        /// https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters
        /// </summary>
        /// <param name="DHparameters">Array of four float parameters</param>
        /// <returns></returns>
        private Matrix4x4 FWDMatrix2(float[] DHparameters)
        {
            return new Matrix4x4(new Vector4(Mathf.Cos(DHparameters[2]), -Mathf.Sin(DHparameters[2]) * Mathf.Cos(DHparameters[0]), Mathf.Sin(DHparameters[2]) * Mathf.Sin(DHparameters[0]), DHparameters[1] * Mathf.Cos(DHparameters[2])),
                                new Vector4(Mathf.Sin(DHparameters[2]) , Mathf.Cos(DHparameters[2]) * Mathf.Cos(DHparameters[0]), -Mathf.Sin(DHparameters[0]) * Mathf.Cos(DHparameters[2]), Mathf.Sin(DHparameters[2]) * DHparameters[1]),
                                new Vector4(0, Mathf.Sin(DHparameters[0]), Mathf.Cos(DHparameters[0]), DHparameters[3]),
                                new Vector4(0, 0, 0, 1));
        }

        private void OnDrawGizmos()
        {
            if (showFKGizmo && robot != null)
            {
                // Draw base frame
                Vector3 basePosition = robot.transform.position;

                // Draw sphere at base frame origin
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(basePosition, gizmoSize);

                // Draw base frame axes (X=red, Y=green, Z=blue)
                Vector3 baseXAxis = robot.transform.right * gizmoSize * 2;
                Vector3 baseYAxis = robot.transform.up * gizmoSize * 2;
                Vector3 baseZAxis = robot.transform.forward * gizmoSize * 2;

                Gizmos.color = Color.red;
                Gizmos.DrawLine(basePosition, basePosition + baseXAxis);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(basePosition, basePosition + baseYAxis);
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(basePosition, basePosition + baseZAxis);

#if UNITY_EDITOR
                Handles.Label(basePosition + Vector3.up * gizmoSize * 0.3f, "Base", new GUIStyle {
                    normal = new GUIStyleState { textColor = Color.cyan },
                    fontSize = 12,
                    fontStyle = FontStyle.Bold
                });
#endif

                // Draw intermediate frames for each joint
                if (dh != null && dh.Count == jointChain.Count && dh.Count > 0)
                {
                    Matrix4x4 accumulatedTransform = Matrix4x4.identity;

                    for (int i = 0; i < dh.Count; i++)
                    {
                        // Compute FK up to joint i
                        accumulatedTransform = accumulatedTransform * FWDMatrix2(dh[i]).transpose;

                        // Extract position and orientation
                        Vector3 framePositionLocal = new Vector3(accumulatedTransform.m03, accumulatedTransform.m13, accumulatedTransform.m23);
                        Vector3 framePosition = robot.transform.TransformPoint(framePositionLocal);

                        Vector3 xAxis = robot.transform.TransformDirection((Vector3)accumulatedTransform.GetColumn(0));
                        Vector3 yAxis = robot.transform.TransformDirection((Vector3)accumulatedTransform.GetColumn(1));
                        Vector3 zAxis = robot.transform.TransformDirection((Vector3)accumulatedTransform.GetColumn(2));

                        // Draw coordinate axes
                        Gizmos.color = Color.red;
                        Gizmos.DrawLine(framePosition, framePosition + xAxis * gizmoSize * 1.5f);
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(framePosition, framePosition + yAxis * gizmoSize * 1.5f);
                        Gizmos.color = Color.blue;
                        Gizmos.DrawLine(framePosition, framePosition + zAxis * gizmoSize * 1.5f);

#if UNITY_EDITOR
                        // Draw frame number label
                        Handles.Label(framePosition + Vector3.up * gizmoSize * 0.3f, $"Frame {i + 1}", new GUIStyle {
                            normal = new GUIStyleState { textColor = Color.white },
                            fontSize = 10
                        });
#endif
                    }
                }

                // Draw end-effector frame with wire sphere
                if (endEffectorPosition != Matrix4x4.zero)
                {
                    // FK position in base frame coordinates
                    Vector3 fkPositionLocal = new Vector3(endEffectorPosition.m03, endEffectorPosition.m13, endEffectorPosition.m23);

                    // Convert to world space
                    Vector3 fkPosition = robot.transform.TransformPoint(fkPositionLocal);

                    // Draw sphere at FK end effector position
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawWireSphere(fkPosition, gizmoSize);

                    // Draw coordinate frame axes (X=red, Y=green, Z=blue)
                    Vector3 xAxis = robot.transform.TransformDirection((Vector3)endEffectorPosition.GetColumn(0));
                    Vector3 yAxis = robot.transform.TransformDirection((Vector3)endEffectorPosition.GetColumn(1));
                    Vector3 zAxis = robot.transform.TransformDirection((Vector3)endEffectorPosition.GetColumn(2));

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
        /// Automatically populates DH parameters by extracting them from the ArticulationBody joint chain.
        /// </summary>
        public void AutoPopulateDHParameters()
        {
            if (jointChain == null || jointChain.Count == 0)
            {
                Debug.LogError("FKRobot: Cannot auto-populate DH parameters - joint chain is empty");
                return;
            }

            // Initialize dh list with proper size
            dh = new List<float[]>();
            for (int i = 0; i < jointChain.Count; i++)
                dh.Add(new float[4]); // [α, a, θ, d]

            // Extract DH parameters using utility class
            DHParameterExtractor.ExtractDHParameters(jointChain, dh);

            // Validate and log results
            if (ValidateDHParameters())
            {
                Debug.Log($"FKRobot: Successfully computed DH parameters for {dh.Count} joints");
                LogDHParameters();
            }
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
