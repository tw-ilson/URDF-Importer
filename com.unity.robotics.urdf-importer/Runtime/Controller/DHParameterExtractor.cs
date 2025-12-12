using UnityEngine;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
using System.IO;
#endif

namespace Unity.Robotics.UrdfImporter.Control
{
    /// <summary>
    /// Utility class for extracting Denavit-Hartenberg parameters from ArticulationBody joint chains.
    /// Implements proper DH frame construction following geometric constraints:
    /// - x_i perpendicular to both z_{i-1} and z_i
    /// - x_i intersects both axes (common normal)
    /// - Origin at intersection of x_i and z_i
    /// </summary>
    public static class DHParameterExtractor
    {
        private const float EPSILON = 0.001f;
        private const float PARALLEL_THRESHOLD = 0.9999f;

#if UNITY_EDITOR
        /// <summary>
        /// Creates a RobotDHParameters ScriptableObject asset from the joint chain and saves it to the Assets folder.
        /// </summary>
        /// <param name="jointChain">List of ArticulationBody joints in kinematic order</param>
        /// <param name="robotName">Name of the robot (used for asset naming)</param>
        /// <param name="savePath">Optional custom save path within Assets folder (default: "Assets/RobotDHParameters/")</param>
        /// <returns>The created RobotDHParameters asset, or null if creation failed</returns>
        public static RobotDHParameters CreateAndSaveDHParametersAsset(
            List<ArticulationBody> jointChain,
            string robotName = "Robot",
            string savePath = "Assets/RobotDHParameters/")
        {
            if (jointChain == null || jointChain.Count == 0)
            {
                Debug.LogError("DHParameterExtractor: Cannot create DH parameters asset - joint chain is empty");
                return null;
            }

            // Create ScriptableObject instance
            RobotDHParameters asset = ScriptableObject.CreateInstance<RobotDHParameters>();
            asset.robotName = robotName;

            // Extract DH parameters into temporary list
            List<float[]> dhParams = new List<float[]>();
            for (int i = 0; i < jointChain.Count; i++)
                dhParams.Add(new float[4]);

            ExtractDHParameters(jointChain, dhParams);

            // Convert to RobotDHParameters format [theta, d, a, alpha]
            asset.dhParameters = new RobotDHParameters.DHParameter[dhParams.Count];
            for (int i = 0; i < dhParams.Count; i++)
            {
                // dhParams is [alpha, a, theta, d], convert to [theta, d, a, alpha]
                asset.dhParameters[i] = new RobotDHParameters.DHParameter(
                    theta: dhParams[i][2],
                    d: dhParams[i][3],
                    a: dhParams[i][1],
                    alpha: dhParams[i][0]
                );
            }

            // Ensure save directory exists
            if (!Directory.Exists(savePath))
            {
                Directory.CreateDirectory(savePath);
            }

            // Generate unique filename
            string assetName = $"{robotName}_DHParameters.asset";
            string assetPath = Path.Combine(savePath, assetName);
            assetPath = AssetDatabase.GenerateUniqueAssetPath(assetPath);

            // Save asset
            AssetDatabase.CreateAsset(asset, assetPath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log($"DHParameterExtractor: Created and saved DH parameters asset at: {assetPath}");

            return asset;
        }
#endif

        /// <summary>
        /// Extracts DH parameters for entire joint chain.
        /// </summary>
        /// <param name="jointChain">List of ArticulationBody joints in kinematic order</param>
        /// <param name="dhParams">Output list of DH parameters [α, a, θ, d] for each joint</param>
        public static void ExtractDHParameters(List<ArticulationBody> jointChain, List<float[]> dhParams)
        {
            if (jointChain == null || jointChain.Count == 0)
            {
                Debug.LogError("DHParameterExtractor: Joint chain is null or empty");
                return;
            }

            if (dhParams == null || dhParams.Count != jointChain.Count)
            {
                Debug.LogError("DHParameterExtractor: dhParams must be initialized with same length as jointChain");
                return;
            }

            // Track DH frame origins (computed geometrically, not from Unity transforms)
            Vector3 dhFrameOrigin_prev = Vector3.zero;

            for (int i = 0; i < jointChain.Count; i++)
            {
                Vector3 z_prev, z_curr;
                Vector3 jointPosition_prev, jointPosition_curr;

                // Get previous joint data (or base frame for first joint)
                if (i == 0)
                {
                    // Base frame: Use robot's parent transform to define base frame
                    // z-axis should represent the fixed base orientation
                    Transform baseTransform = jointChain[0].transform.parent;
                    if (baseTransform != null)
                    {
                        // Use parent's forward direction (Unity Z) as base frame z-axis
                        // This aligns with typical robot mounting conventions
                        z_prev = baseTransform.forward;
                        dhFrameOrigin_prev = baseTransform.position;
                        jointPosition_prev = baseTransform.position;
                    }
                    else
                    {
                        // Fallback: use world frame
                        z_prev = Vector3.forward;
                        dhFrameOrigin_prev = jointChain[0].transform.position;
                        jointPosition_prev = dhFrameOrigin_prev;
                    }
                }
                else
                {
                    z_prev = GetJointAxis(jointChain[i - 1]);
                    jointPosition_prev = jointChain[i - 1].transform.position;
                    // dhFrameOrigin_prev is already set from previous iteration
                }

                // Get current joint axis and position
                z_curr = GetJointAxis(jointChain[i]);
                jointPosition_curr = jointChain[i].transform.position;

                // Compute DH parameters and get the geometric frame origin for frame {i}
                float a, alpha, d, theta;
                Vector3 dhFrameOrigin_curr;
                ComputeDHForJointPair(z_prev, z_curr, dhFrameOrigin_prev, jointPosition_prev, jointPosition_curr,
                                     out a, out alpha, out d, out theta, out dhFrameOrigin_curr);

                // Store in [α, a, θ, d] order
                dhParams[i][0] = alpha;
                dhParams[i][1] = a;
                dhParams[i][2] = theta;
                dhParams[i][3] = d;

                // Update for next iteration
                dhFrameOrigin_prev = dhFrameOrigin_curr;
            }
        }

        /// <summary>
        /// Extracts joint axis direction from ArticulationBody.
        /// Based on how UrdfJointRevolute sets anchorRotation (line 162):
        /// Motion.SetFromToRotation(Vector3.right, -axisUnity)
        /// </summary>
        private static Vector3 GetJointAxis(ArticulationBody joint)
        {
            if (joint == null)
            {
                Debug.LogWarning("DHParameterExtractor: Null joint, returning Vector3.right");
                return Vector3.right;
            }

            // ArticulationBody aligns joint axis to local X-axis
            // anchorRotation rotates from (1,0,0) to -axisUnity
            // So joint axis = -(anchorRotation * Vector3.right)
            Vector3 localAxis = joint.anchorRotation * Vector3.right;
            Vector3 worldAxis = joint.transform.TransformDirection(localAxis);

            // Negate to get actual joint axis (due to SetFromToRotation using -axisUnity)
            return -worldAxis.normalized;
        }

        /// <summary>
        /// Computes DH parameters for a pair of consecutive joints.
        /// Modified DH convention: Rot_z(θ) → Trans_z(d) → Trans_x(a) → Rot_x(α)
        /// </summary>
        /// <param name="z_prev">Previous joint axis direction</param>
        /// <param name="z_curr">Current joint axis direction</param>
        /// <param name="dhFrameOrigin_prev">DH frame origin for frame {i-1}</param>
        /// <param name="jointPosition_prev">Unity transform position of previous joint (used to define axis line)</param>
        /// <param name="jointPosition_curr">Unity transform position of current joint (used to define axis line)</param>
        /// <param name="dhFrameOrigin_curr">Output: DH frame origin for frame {i} (where x_i intersects z_i)</param>
        private static void ComputeDHForJointPair(
            Vector3 z_prev, Vector3 z_curr,
            Vector3 dhFrameOrigin_prev, Vector3 jointPosition_prev, Vector3 jointPosition_curr,
            out float a, out float alpha, out float d, out float theta,
            out Vector3 dhFrameOrigin_curr)
        {
            Vector3 x_i;
            float distanceBetweenAxes;
            Vector3 pointOnPrevAxis;
            Vector3 pointOnCurrAxis;

            // Compute common normal between z_prev and z_curr
            // Use joint positions to define the axis lines in space
            x_i = ComputeCommonNormal(z_prev, z_curr, jointPosition_prev, jointPosition_curr,
                                     out distanceBetweenAxes, out pointOnPrevAxis, out pointOnCurrAxis);

            // DH frame {i} origin is where x_i intersects z_i (the current joint axis)
            dhFrameOrigin_curr = pointOnCurrAxis;

            // a: link length (distance along x_i between the two z-axes)
            a = distanceBetweenAxes;

            // α: link twist (angle from z_prev to z_curr about x_i)
            alpha = ComputeTwistAngle(z_prev, z_curr, x_i);

            // d: link offset (distance along z_prev from DH frame {i-1} origin to where x_i intersects z_prev)
            Vector3 toIntersection = pointOnPrevAxis - dhFrameOrigin_prev;
            d = Vector3.Dot(toIntersection, z_prev);

            // θ: joint angle (rotation about z_prev)
            // For initial configuration, this is typically 0 or determined by current joint position
            // We set it to 0 here; it will be updated by PopulateDHparameters() at runtime
            theta = 0f;
        }

        /// <summary>
        /// Computes the common normal (perpendicular line) between two spatial axes.
        /// Returns the direction of the common normal and its length.
        /// </summary>
        private static Vector3 ComputeCommonNormal(
            Vector3 z_prev, Vector3 z_curr,
            Vector3 origin_prev, Vector3 origin_curr,
            out float distance, out Vector3 pointOnPrev, out Vector3 pointOnCurr)
        {
            z_prev = z_prev.normalized;
            z_curr = z_curr.normalized;

            float dotProduct = Vector3.Dot(z_prev, z_curr);
            bool areParallel = Mathf.Abs(dotProduct) > PARALLEL_THRESHOLD;

            if (areParallel)
            {
                // Parallel axes: choose x_i perpendicular to both
                // Use connection vector to determine direction
                Vector3 connection = origin_curr - origin_prev;

                // Project connection onto plane perpendicular to z_prev
                Vector3 perpComponent = connection - Vector3.Dot(connection, z_prev) * z_prev;

                if (perpComponent.magnitude < EPSILON)
                {
                    // Connection is along z_prev, choose arbitrary perpendicular
                    perpComponent = FindPerpendicularVector(z_prev);
                }

                Vector3 x_i = perpComponent.normalized;
                distance = perpComponent.magnitude;
                pointOnPrev = origin_prev;
                pointOnCurr = origin_prev + perpComponent;

                return x_i;
            }
            else
            {
                // Skew or intersecting axes: compute closest points
                // Using standard line-line closest point algorithm
                Vector3 w = origin_prev - origin_curr;

                float a = Vector3.Dot(z_prev, z_prev);
                float b = Vector3.Dot(z_prev, z_curr);
                float c = Vector3.Dot(z_curr, z_curr);
                float d_param = Vector3.Dot(z_prev, w);
                float e = Vector3.Dot(z_curr, w);

                float denom = a * c - b * b;

                if (Mathf.Abs(denom) < EPSILON)
                {
                    // Degenerate case (shouldn't happen if not parallel)
                    distance = 0f;
                    pointOnPrev = origin_prev;
                    pointOnCurr = origin_curr;
                    return FindPerpendicularVector(z_prev);
                }

                float s = (b * e - c * d_param) / denom;
                float t = (a * e - b * d_param) / denom;

                pointOnPrev = origin_prev + s * z_prev;
                pointOnCurr = origin_curr + t * z_curr;

                Vector3 commonNormal = pointOnCurr - pointOnPrev;
                distance = commonNormal.magnitude;

                if (distance < EPSILON)
                {
                    // Axes intersect: a = 0, choose x_i perpendicular to both
                    Vector3 cross = Vector3.Cross(z_prev, z_curr);
                    if (cross.magnitude < EPSILON)
                    {
                        return FindPerpendicularVector(z_prev);
                    }
                    return cross.normalized;
                }

                return commonNormal.normalized;
            }
        }

        /// <summary>
        /// Computes twist angle (α) from z_prev to z_curr about x_i axis.
        /// Uses signed angle for proper orientation.
        /// </summary>
        private static float ComputeTwistAngle(Vector3 z_prev, Vector3 z_curr, Vector3 x_i)
        {
            // Project both z-axes onto plane perpendicular to x_i
            Vector3 z_prev_proj = z_prev - Vector3.Dot(z_prev, x_i) * x_i;
            Vector3 z_curr_proj = z_curr - Vector3.Dot(z_curr, x_i) * x_i;

            if (z_prev_proj.magnitude < EPSILON || z_curr_proj.magnitude < EPSILON)
            {
                // Axes are parallel to x_i (shouldn't happen in valid DH)
                return 0f;
            }

            z_prev_proj.Normalize();
            z_curr_proj.Normalize();

            // Compute signed angle using atan2
            float cosAngle = Vector3.Dot(z_prev_proj, z_curr_proj);
            Vector3 cross = Vector3.Cross(z_prev_proj, z_curr_proj);
            float sinAngle = Vector3.Dot(cross, x_i);

            return Mathf.Atan2(sinAngle, cosAngle);
        }

        /// <summary>
        /// Finds an arbitrary vector perpendicular to the given vector.
        /// </summary>
        private static Vector3 FindPerpendicularVector(Vector3 v)
        {
            v = v.normalized;

            // Choose axis least aligned with v
            Vector3 candidate = Mathf.Abs(Vector3.Dot(v, Vector3.right)) < 0.9f
                ? Vector3.right
                : Vector3.up;

            return Vector3.Cross(v, candidate).normalized;
        }
    }
}
