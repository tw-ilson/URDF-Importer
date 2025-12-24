using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

namespace Unity.Robotics.UrdfImporter.Editor
{
    [CustomEditor(typeof(FKRobot), true)]
    public class UrdfFkrobotEditor : UnityEditor.Editor
    {

        private FKRobot fkrobot;

        private SerializedProperty currentAngles;
        private SerializedProperty rotationMatrix;
        private SerializedProperty autoComputeDH;
        private SerializedProperty dhParametersAsset;
        private SerializedProperty showFKGizmo;
        private SerializedProperty visualizationSphere;
        private SerializedProperty gizmoSize;

        private void OnEnable()
        {
            currentAngles = serializedObject.FindProperty("currentAngles");
            rotationMatrix = serializedObject.FindProperty("endEffectorPosition");
            autoComputeDH = serializedObject.FindProperty("autoComputeDH");
            dhParametersAsset = serializedObject.FindProperty("dhParametersAsset");
            showFKGizmo = serializedObject.FindProperty("showFKGizmo");
            visualizationSphere = serializedObject.FindProperty("visualizationSphere");
            gizmoSize = serializedObject.FindProperty("gizmoSize");
        }
        public override void OnInspectorGUI()
        {
            fkrobot = (FKRobot)target;

            serializedObject.Update();

            // DH Parameter Configuration
            EditorGUILayout.PropertyField(dhParametersAsset);

            GUILayout.Space(10);

            // FK Visualization
            EditorGUILayout.PropertyField(showFKGizmo);
            EditorGUILayout.PropertyField(visualizationSphere);
            EditorGUILayout.PropertyField(gizmoSize);

            GUILayout.Space(10);

            // Current State (Read-only)
            EditorGUILayout.PropertyField(currentAngles);
            EditorGUILayout.PropertyField(rotationMatrix);

            serializedObject.ApplyModifiedProperties();

            GUILayout.Space(10);

            // Manual DH Parameter Entry
            EditorGUI.BeginDisabledGroup(fkrobot.dhParametersAsset != null);
            if (GUILayout.Button("Add DH Parameters Manually"))
            {
                AddDhParameterWindow window = (AddDhParameterWindow)EditorWindow.GetWindow(typeof(AddDhParameterWindow));
                window.script = fkrobot;
                window.minSize = new Vector2(500, 200);
                window.GetEditorPrefs();
                window.Show();
            }
            EditorGUI.EndDisabledGroup();
        }


    }
}
