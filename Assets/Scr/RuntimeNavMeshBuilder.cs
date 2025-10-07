using UnityEngine;
using Unity.AI.Navigation;

public class RuntimeNavMeshBuilder : MonoBehaviour
{
    public NavMeshSurface surface;

    public void BuildNavMesh()
    {
            surface.BuildNavMesh();
    }
}
