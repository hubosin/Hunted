using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class CityGenerator : MonoBehaviour
{
    [Header("Terrain & Prefabs")]
    public Terrain terrain;
    public GameObject[] buildingPrefabs;

    [Header("Voronoi Road Network")]
    [SerializeField] private int voronoiSites = 40;
    [SerializeField] private float cityRadius = 400f;
    [SerializeField] private float mainRoadWidth = 20f;
    [SerializeField] private float sideRoadWidth = 12f;
    [SerializeField] private int relaxationIterations = 2;

    [Header("City Layout")]
    [SerializeField] private Vector3 cityCenter = Vector3.zero;
    [SerializeField] private float hiveCenterRadius = 60f;
    [SerializeField] private bool createCentralPlaza = true;

    [Header("Building Settings")]
    [SerializeField] private float minBuildingSpacing = 6f;
    [SerializeField] private float buildingDensity = 0.6f;
    [SerializeField] private float groundOffset = 0f;
    [Range(0f, 45f)] public float maxSlope = 15f;

    [Header("Player Safe Zone")]
    [Tooltip("No buildings spawn within this radius")]
    public float noBuildRadius = 50f;

    // Voronoi structures
    [System.Serializable]
    public class VoronoiSite
    {
        public Vector3 position;
        public List<VoronoiSite> neighbors = new List<VoronoiSite>();
        public bool isHiveCenter;
    }

    [System.Serializable]
    public class RoadSegment
    {
        public Vector3 start;
        public Vector3 end;
        public float width;
        public bool isMainRoad;

        public RoadSegment(Vector3 s, Vector3 e, float w, bool main = false)
        {
            start = s;
            end = e;
            width = w;
            isMainRoad = main;
        }

        public bool ContainsPoint(Vector3 point, float extraMargin = 0f)
        {
            float margin = width / 2f + extraMargin;
            return DistanceToLineSegment(point, start, end) < margin;
        }

        private float DistanceToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 line = lineEnd - lineStart;
            float len = line.magnitude;
            if (len < 0.001f) return Vector3.Distance(point, lineStart);

            float t = Mathf.Clamp01(Vector3.Dot(point - lineStart, line) / (len * len));
            Vector3 projection = lineStart + t * line;
            return Vector3.Distance(point, projection);
        }

        public List<Vector3> GetWaypoints(float spacing = 30f)
        {
            List<Vector3> waypoints = new List<Vector3>();
            float dist = Vector3.Distance(start, end);
            int count = Mathf.Max(2, Mathf.CeilToInt(dist / spacing));

            for (int i = 0; i <= count; i++)
            {
                float t = i / (float)count;
                waypoints.Add(Vector3.Lerp(start, end, t));
            }

            return waypoints;
        }
    }

    // Public data for HiveMind
    [HideInInspector] public List<RoadSegment> roads = new List<RoadSegment>();
    [HideInInspector] public List<VoronoiSite> sites = new List<VoronoiSite>();
    [HideInInspector] public Vector3 hiveCenter;
    [HideInInspector] public VoronoiSite hiveSite;

    private bool cityGenerated = false;
    public event System.Action OnCityGenerated;

    void Start()
    {
        if (cityCenter == Vector3.zero)
            cityCenter = transform.position;

        hiveCenter = cityCenter;

        StartCoroutine(WaitForLoadAndGenerate());
    }

    System.Collections.IEnumerator WaitForLoadAndGenerate()
    {
        while (!IsSceneReady())
            yield return null;

        yield return new WaitForSeconds(0.5f);
        GenerateCity();
    }

    bool IsSceneReady()
    {
        if (!terrain) return false;
        if (terrain.terrainData == null) return false;
        if (buildingPrefabs == null || buildingPrefabs.Length == 0) return false;
        if (!gameObject.scene.isLoaded) return false;
        return true;
    }

    void GenerateCity()
    {
        if (cityGenerated) return;
        cityGenerated = true;

        if (!terrain || buildingPrefabs.Length == 0)
        {
            Debug.LogWarning("Missing terrain or building prefabs!");
            return;
        }

        Debug.Log("=== Starting Voronoi City Generation ===");

        // Step 1: Generate Voronoi diagram
        GenerateVoronoiSites();

        // Step 2: Create roads from Voronoi edges (Delaunay triangulation)
        GenerateRoadsFromVoronoi();

        // Step 3: Place buildings in cells
        PlaceBuildings();

        Debug.Log($"City generated: {roads.Count} roads, {sites.Count} districts");
        OnCityGenerated?.Invoke();
    }

    private void GenerateVoronoiSites()
    {
        sites.Clear();

        // Generate random points in circle
        for (int i = 0; i < voronoiSites; i++)
        {
            Vector3 pos;

            if (i == 0)
            {
                // First site is always the hive center
                pos = cityCenter;
            }
            else
            {
                // Random position in circle using uniform distribution
                float angle = Random.Range(0f, 360f);
                float radius = Mathf.Sqrt(Random.Range(0f, 1f)) * cityRadius;
                pos = cityCenter + Quaternion.Euler(0, angle, 0) * Vector3.forward * radius;
            }

            VoronoiSite site = new VoronoiSite
            {
                position = pos,
                isHiveCenter = (i == 0)
            };

            sites.Add(site);
        }

        hiveSite = sites[0];
        hiveCenter = hiveSite.position;

        // Lloyd's relaxation for better distribution
        for (int iter = 0; iter < relaxationIterations; iter++)
        {
            RelaxVoronoiSites();
        }

        // Snap to terrain
        SnapSitesToTerrain();

        // Build Delaunay triangulation (neighbor connections)
        BuildDelaunayTriangulation();

        Debug.Log($"Generated {sites.Count} Voronoi sites");
    }

    private void RelaxVoronoiSites()
    {
        // Lloyd's relaxation: move each point to centroid of its Voronoi cell
        // Simplified version - just average with neighbors

        List<Vector3> newPositions = new List<Vector3>();

        foreach (var site in sites)
        {
            if (site.isHiveCenter)
            {
                newPositions.Add(site.position); // Keep hive center fixed
                continue;
            }

            // Find nearby sites
            var nearby = sites
                .Where(s => s != site)
                .OrderBy(s => Vector3.Distance(s.position, site.position))
                .Take(6)
                .ToList();

            if (nearby.Count > 0)
            {
                Vector3 centroid = site.position;
                foreach (var n in nearby)
                {
                    centroid += n.position;
                }
                centroid /= (nearby.Count + 1);
                newPositions.Add(centroid);
            }
            else
            {
                newPositions.Add(site.position);
            }
        }

        for (int i = 0; i < sites.Count; i++)
        {
            sites[i].position = newPositions[i];
        }
    }

    private void BuildDelaunayTriangulation()
    {
        // Simple approach: connect each site to its N nearest neighbors
        // This creates a graph similar to Delaunay triangulation

        foreach (var site in sites)
        {
            site.neighbors.Clear();

            var nearest = sites
                .Where(s => s != site)
                .OrderBy(s => Vector3.Distance(s.position, site.position))
                .Take(6) // Connect to 6 nearest
                .ToList();

            foreach (var neighbor in nearest)
            {
                if (!site.neighbors.Contains(neighbor))
                {
                    site.neighbors.Add(neighbor);
                }
                if (!neighbor.neighbors.Contains(site))
                {
                    neighbor.neighbors.Add(site);
                }
            }
        }
    }

    private void GenerateRoadsFromVoronoi()
    {
        roads.Clear();
        HashSet<string> processedEdges = new HashSet<string>();

        foreach (var site in sites)
        {
            foreach (var neighbor in site.neighbors)
            {
                // Create unique edge ID
                string edgeID = GetEdgeID(site, neighbor);
                if (processedEdges.Contains(edgeID)) continue;
                processedEdges.Add(edgeID);

                // Road connects the two sites
                Vector3 start = site.position;
                Vector3 end = neighbor.position;

                // Determine if main road (connected to hive center or both are important)
                bool isMainRoad = site.isHiveCenter || neighbor.isHiveCenter;
                float width = isMainRoad ? mainRoadWidth : sideRoadWidth;

                RoadSegment road = new RoadSegment(start, end, width, isMainRoad);
                roads.Add(road);
            }
        }

        Debug.Log($"Generated {roads.Count} road segments from Voronoi diagram");
    }

    private string GetEdgeID(VoronoiSite a, VoronoiSite b)
    {
        // Create consistent edge ID regardless of order
        int idA = sites.IndexOf(a);
        int idB = sites.IndexOf(b);
        int min = Mathf.Min(idA, idB);
        int max = Mathf.Max(idA, idB);
        return $"{min}-{max}";
    }

    private void SnapSitesToTerrain()
    {
        TerrainData data = terrain.terrainData;
        Vector3 terrainPos = terrain.transform.position;

        for (int i = 0; i < sites.Count; i++)
        {
            Vector3 pos = sites[i].position;
            float relX = (pos.x - terrainPos.x) / data.size.x;
            float relZ = (pos.z - terrainPos.z) / data.size.z;

            if (relX >= 0 && relX <= 1 && relZ >= 0 && relZ <= 1)
            {
                float height = data.GetInterpolatedHeight(relX, relZ);
                sites[i].position = new Vector3(pos.x, terrainPos.y + height + 0.5f, pos.z);
            }
        }

        // Update hive center
        hiveCenter = hiveSite.position;
    }

    private void PlaceBuildings()
    {
        TerrainData data = terrain.terrainData;
        Vector3 terrainPos = terrain.transform.position;
        int totalBuildings = 0;

        // For each Voronoi cell, place buildings
        foreach (var site in sites)
        {
            // Skip hive center area
            if (site.isHiveCenter && createCentralPlaza) continue;

            // Calculate cell area (approximate)
            float avgNeighborDist = 0;
            if (site.neighbors.Count > 0)
            {
                avgNeighborDist = site.neighbors.Average(n => Vector3.Distance(site.position, n.position));
            }
            else
            {
                avgNeighborDist = 50f;
            }

            // Determine building count for this cell
            float cellArea = avgNeighborDist * avgNeighborDist * 0.5f;
            int buildingsInCell = Mathf.FloorToInt(cellArea / 300f * buildingDensity);

            int placed = 0;
            int attempts = 0;
            int maxAttempts = buildingsInCell * 8;

            while (placed < buildingsInCell && attempts < maxAttempts)
            {
                attempts++;

                // Random position within cell (around site position)
                Vector3 offset = Random.insideUnitSphere * (avgNeighborDist * 0.4f);
                offset.y = 0;
                Vector3 pos = site.position + offset;

                // Check if too close to roads
                bool tooCloseToRoad = false;
                foreach (var road in roads)
                {
                    if (road.ContainsPoint(pos, minBuildingSpacing))
                    {
                        tooCloseToRoad = true;
                        break;
                    }
                }

                if (tooCloseToRoad) continue;

                // Check player safe zone
                if (Vector3.Distance(pos, transform.position) < noBuildRadius)
                    continue;

                // Check hive safe zone
                if (Vector3.Distance(pos, hiveCenter) < hiveCenterRadius)
                    continue;

                // Get terrain height
                float relX = (pos.x - terrainPos.x) / data.size.x;
                float relZ = (pos.z - terrainPos.z) / data.size.z;

                if (relX < 0 || relX > 1 || relZ < 0 || relZ > 1) continue;

                float height = data.GetInterpolatedHeight(relX, relZ);
                Vector3 worldPos = new Vector3(pos.x, terrainPos.y + height, pos.z);

                // Check if flat enough
                if (!IsFlatEnough(data, pos.x - terrainPos.x, pos.z - terrainPos.z, maxSlope))
                    continue;

                // Place building
                GameObject prefab = buildingPrefabs[Random.Range(0, buildingPrefabs.Length)];
                Quaternion rot = Quaternion.Euler(0, Random.Range(0, 360f), 0);
                float scale = Random.Range(0.8f, 1.3f);

                GameObject building = Instantiate(prefab, worldPos, rot, transform);
                building.transform.localScale *= scale;

                // Ground alignment
                Renderer rend = building.GetComponentInChildren<Renderer>();
                if (rend)
                {
                    float bottomY = rend.bounds.min.y;
                    float offsetY = (worldPos.y + groundOffset) - bottomY;
                    building.transform.position += Vector3.up * offsetY;
                }

                placed++;
                totalBuildings++;
            }
        }

        Debug.Log($"Placed {totalBuildings} buildings across {sites.Count} Voronoi cells");
    }

    bool IsFlatEnough(TerrainData data, float x, float z, float maxSlope)
    {
        float step = 1f;
        float h = data.GetInterpolatedHeight(x / data.size.x, z / data.size.z);
        float h1 = data.GetInterpolatedHeight((x + step) / data.size.x, z / data.size.z);
        float h2 = data.GetInterpolatedHeight((x - step) / data.size.x, z / data.size.z);
        float h3 = data.GetInterpolatedHeight(x / data.size.x, (z + step) / data.size.z);
        float h4 = data.GetInterpolatedHeight(x / data.size.x, (z - step) / data.size.z);

        float maxDiff = Mathf.Max(Mathf.Abs(h - h1), Mathf.Abs(h - h2), Mathf.Abs(h - h3), Mathf.Abs(h - h4));
        float slopeDeg = Mathf.Atan(maxDiff / step) * Mathf.Rad2Deg;

        return slopeDeg <= maxSlope;
    }

    // Public API for HiveMind
    public List<Vector3> GetAllRoadWaypoints(float spacing = 30f)
    {
        List<Vector3> waypoints = new List<Vector3>();
        foreach (var road in roads)
        {
            waypoints.AddRange(road.GetWaypoints(spacing));
        }
        return waypoints;
    }

    public List<Vector3> GetRoadIntersections()
    {
        // Intersections are basically the Voronoi sites
        return sites.Select(s => s.position).ToList();
    }

    public bool IsPointOnRoad(Vector3 point, float tolerance = 2f)
    {
        foreach (var road in roads)
        {
            if (road.ContainsPoint(point, tolerance))
                return true;
        }
        return false;
    }

    public VoronoiSite GetNearestSite(Vector3 position)
    {
        return sites.OrderBy(s => Vector3.Distance(s.position, position)).FirstOrDefault();
    }

    void OnDrawGizmosSelected()
    {
        // Draw player safe zone
        Gizmos.color = new Color(1, 0, 0, 0.25f);
        Gizmos.DrawSphere(transform.position, noBuildRadius);

        // Draw hive center
        Gizmos.color = new Color(1, 0, 1, 0.5f);
        Gizmos.DrawWireSphere(hiveCenter, hiveCenterRadius);

        // Draw Voronoi sites
        if (sites != null && sites.Count > 0)
        {
            foreach (var site in sites)
            {
                Gizmos.color = site.isHiveCenter ? Color.magenta : Color.blue;
                Gizmos.DrawSphere(site.position, site.isHiveCenter ? 5f : 3f);
            }
        }

        // Draw roads
        if (roads != null && roads.Count > 0)
        {
            foreach (var road in roads)
            {
                Gizmos.color = road.isMainRoad ? Color.yellow : Color.cyan;
                Gizmos.DrawLine(road.start, road.end);

                // Draw road width
                Vector3 dir = (road.end - road.start).normalized;
                Vector3 perp = new Vector3(-dir.z, 0, dir.x) * road.width * 0.5f;
                Gizmos.color = road.isMainRoad ? new Color(1, 1, 0, 0.2f) : new Color(0, 1, 1, 0.1f);
                Gizmos.DrawLine(road.start + perp, road.end + perp);
                Gizmos.DrawLine(road.start - perp, road.end - perp);
            }
        }
    }
}