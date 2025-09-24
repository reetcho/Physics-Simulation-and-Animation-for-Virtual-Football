using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

namespace Utils
{
    public class DisplayShotComputation : MonoBehaviour
    {
        [SerializeField] private Material lineMaterial;
        [SerializeField] private float lineWidth = 0.035f;

        private List<LineRenderer> _lineRenderers = new();

        public void DisplayComputedTrajectories(List<List<Vector3>> precalculationTrajectories)
        {
            if (!Application.isPlaying)
                return;

            int count = precalculationTrajectories.Count;

            while (_lineRenderers.Count < count)
            {
                var lrObj = new GameObject("TrajectoryLine");
                lrObj.transform.SetParent(transform);
                var lr = lrObj.AddComponent<LineRenderer>();
                lr.material = new Material(lineMaterial);
                lr.widthMultiplier = lineWidth;
                lr.useWorldSpace = true;
                lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                lr.receiveShadows = false;
                _lineRenderers.Add(lr);
            }

            for (int i = 0; i < count; i++)
            {
                float t = (float)(i + 1) / count;
                Color c = Color.Lerp(Color.green, Color.blue, t);

                LineRenderer lr = _lineRenderers[i];
                lr.startColor = lr.endColor = c;

                Vector3[] pts = precalculationTrajectories[i].ToArray();
                lr.positionCount = pts.Length;
                lr.SetPositions(pts);
            }

            for (int i = count; i < _lineRenderers.Count; i++)
                _lineRenderers[i].positionCount = 0;
        }
        
        public void DeleteComputedTrajectories()
        {
            _lineRenderers.Clear();
            
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                GameObject child = transform.GetChild(i).gameObject;
                Destroy(child);
            }
        }
    }
}