using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class HiveMind : MonoBehaviour
{
    [System.Serializable]
    public class PatrolNode
    {
        public Vector3 position;
        [System.NonSerialized]
        public List<PatrolNode> connections = new List<PatrolNode>();
        public bool isImportant;
        public NodeType type;
        public float heat;
        public float lastVisitTime;
        public bool isOnRoad;
        public float creationTime; // Track when node was created

        public enum NodeType { Standard, Chokepoint, Vantage, Cover, Junction, RoadNode, Intersection }

        public PatrolNode(Vector3 pos, NodeType nodeType = NodeType.Standard)
        {
            position = pos;
            type = nodeType;
            heat = 0f;
            connections = new List<PatrolNode>();
            isOnRoad = (nodeType == NodeType.RoadNode || nodeType == NodeType.Intersection);
            creationTime = Time.time;
        }
    }

    [Header("References")]
    [SerializeField] private GameObject tankPrefab;
    [SerializeField] private GameObject dronePrefab;
    [SerializeField] private Transform player;
    [SerializeField] private CityGenerator cityGenerator;
    [SerializeField] private Terrain terrain;

    [Header("Initial Spawn Settings")]
    [SerializeField] private int initialTankCount = 3;
    [SerializeField] private int initialDroneCount = 2;

    [Header("Organic Network Growth")]
    [SerializeField] private int initialNodeCount = 5; // Start small!
    [SerializeField] private float nodeGrowthInterval = 5f; // How often to add nodes
    [SerializeField] private int nodesPerGrowth = 2; // How many nodes per growth cycle
    [SerializeField] private float preferRoadGrowth = 0.75f; // 75% chance to grow along roads
    [SerializeField] private float growthDistance = 40f; // How far from existing nodes
    [SerializeField] private float maxGrowthRadius = 300f; // Max distance from hive center

    [Header("Network Parameters")]
    [SerializeField] private float connectionRadius = 60f;
    [SerializeField] private int maxConnectionsPerNode = 5;
    [SerializeField] private LayerMask environmentLayer = -1;

    [Header("Dynamic Expansion")]
    [SerializeField] private float alertExpansionBoost = 3f; // Grow faster when player detected
    [SerializeField] private int maxNodes = 400;
    [SerializeField] private bool expandTowardPlayer = true;
    [SerializeField] private bool expandTowardHotspots = true;

    [Header("Reinforcement System")]
    [SerializeField] private bool enableReinforcements = true;
    [SerializeField] private float reinforcementInterval = 45f;
    [SerializeField] private int maxEnemies = 50;
    [SerializeField] private float spawnDistanceFromPlayer = 80f;

    [Header("Difficulty Scaling")]
    [SerializeField] private bool enableScaling = true;
    [SerializeField] private float scalingInterval = 60f;

    [Header("Player Learning")]
    [SerializeField] private bool enablePlayerLearning = true;
    [SerializeField] private float playerTrackingInterval = 2f;
    [SerializeField] private float visitFrequencyRadius = 25f;
    [SerializeField] private int maxTrackedLocations = 100;
    [SerializeField] private int hotspotThreshold = 5;

    [Header("Coordination")]
    [SerializeField] private float alertRadius = 50f;
    [SerializeField] private float heatDecayRate = 0.1f;

    [Header("Debug")]
    [SerializeField] private bool drawDebugGraph = true;
    [SerializeField] private bool drawHeatMap = true;
    [SerializeField] private bool showGrowthAnimation = true;

    // Internal state
    private List<PatrolNode> patrolNodes = new List<PatrolNode>();
    private List<GameObject> enemies = new List<GameObject>();
    private float nextGrowthTime;
    private float nextReinforcementTime;
    private float nextScalingTime;
    private bool playerDetected = false;
    private Vector3 lastPlayerSpottedPos;
    private float lastPlayerSpottedTime;
    private int currentDifficulty = 1;
    private Vector3 hiveCenter;

    // Player learning
    private List<PlayerVisit> playerVisitHistory = new List<PlayerVisit>();
    private float nextPlayerTrackTime;
    private Vector3 lastTrackedPlayerPos;

    [System.Serializable]
    private class PlayerVisit
    {
        public Vector3 position;
        public float timestamp;
        public int visitCount;

        public PlayerVisit(Vector3 pos, float time)
        {
            position = pos;
            timestamp = time;
            visitCount = 1;
        }
    }

    void Start()
    {
        if (cityGenerator != null)
        {
            cityGenerator.OnCityGenerated += OnCityReady;
        }
        else
        {
            Debug.LogError("CityGenerator reference missing!");
        }
    }

    private void OnCityReady()
    {
        hiveCenter = cityGenerator.hiveCenter;
        InitializeHivemind();
    }

    private void InitializeHivemind()
    {
        // Start with just the hive center and a few nearby nodes
        CreateInitialNetwork();
        SpawnInitialEnemies();

        nextGrowthTime = Time.time + nodeGrowthInterval;
        nextReinforcementTime = Time.time + reinforcementInterval;
        nextScalingTime = Time.time + scalingInterval;
        nextPlayerTrackTime = Time.time + playerTrackingInterval;

        Debug.Log($"HiveMind initialized with {patrolNodes.Count} initial nodes at {hiveCenter}");
    }

    void Update()
    {
        if (patrolNodes.Count == 0) return;

        DecayHeat();

        if (enablePlayerLearning && Time.time >= nextPlayerTrackTime)
        {
            TrackPlayerPosition();
            nextPlayerTrackTime = Time.time + playerTrackingInterval;
        }

        // Organic network growth
        if (Time.time >= nextGrowthTime && patrolNodes.Count < maxNodes)
        {
            GrowNetwork();

            // Adjust growth rate based on alert status
            float growthRate = playerDetected ? (nodeGrowthInterval / alertExpansionBoost) : nodeGrowthInterval;
            nextGrowthTime = Time.time + growthRate;
        }

        if (enableReinforcements && Time.time >= nextReinforcementTime)
        {
            SpawnReinforcements();
            nextReinforcementTime = Time.time + reinforcementInterval;
        }

        if (enableScaling && Time.time >= nextScalingTime)
        {
            IncreaseDifficulty();
            nextScalingTime = Time.time + scalingInterval;
        }

        CleanupDestroyedEnemies();
    }

    private void CreateInitialNetwork()
    {
        patrolNodes.Clear();

        // Create central hive node
        PatrolNode hiveNode = new PatrolNode(hiveCenter, PatrolNode.NodeType.Junction)
        {
            isImportant = true,
            isOnRoad = false
        };
        patrolNodes.Add(hiveNode);

        // Create initial ring of nodes around hive
        for (int i = 0; i < initialNodeCount; i++)
        {
            float angle = (360f / initialNodeCount) * i;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            Vector3 pos = hiveCenter + direction * growthDistance;
            pos = SnapToTerrain(pos);

            // Check if on road
            bool onRoad = cityGenerator.IsPointOnRoad(pos, 5f);
            PatrolNode.NodeType nodeType = onRoad ? PatrolNode.NodeType.RoadNode : PatrolNode.NodeType.Standard;

            PatrolNode node = new PatrolNode(pos, nodeType)
            {
                isOnRoad = onRoad
            };

            // Connect to hive center
            ConnectTwoNodes(hiveNode, node);
            patrolNodes.Add(node);
        }

        Debug.Log($"Created initial network: {patrolNodes.Count} nodes");
    }

    private void GrowNetwork()
    {
        // Organic growth: expand from frontier nodes
        int nodesToAdd = Mathf.Min(nodesPerGrowth, maxNodes - patrolNodes.Count);

        for (int i = 0; i < nodesToAdd; i++)
        {
            PatrolNode growthPoint = SelectGrowthPoint();
            if (growthPoint == null) continue;

            Vector3 newPos = FindNewNodePosition(growthPoint);
            if (newPos == Vector3.zero) continue;

            // Check distance from hive center
            if (Vector3.Distance(newPos, hiveCenter) > maxGrowthRadius) continue;

            newPos = SnapToTerrain(newPos);

            // Determine node type
            bool onRoad = cityGenerator.IsPointOnRoad(newPos, 3f);
            PatrolNode.NodeType nodeType = DetermineNodeType(newPos, onRoad);

            PatrolNode newNode = new PatrolNode(newPos, nodeType)
            {
                isOnRoad = onRoad,
                isImportant = (nodeType == PatrolNode.NodeType.Intersection || nodeType == PatrolNode.NodeType.Junction)
            };

            // Connect to growth point
            ConnectTwoNodes(growthPoint, newNode);

            // Connect to other nearby nodes
            ConnectToNearbyNodes(newNode);

            patrolNodes.Add(newNode);

            Debug.Log($"Network grew: {patrolNodes.Count} nodes (on road: {onRoad}, type: {nodeType})");
        }
    }

    private PatrolNode SelectGrowthPoint()
    {
        List<PatrolNode> candidates = new List<PatrolNode>();

        // Priority 1: Expand toward player hotspots
        if (expandTowardHotspots && playerVisitHistory.Count > 0)
        {
            var hotspot = GetMostVisitedLocation();
            if (hotspot != null && hotspot.visitCount >= hotspotThreshold)
            {
                PatrolNode nearest = GetClosestNode(hotspot.position);
                if (nearest != null && Vector3.Distance(nearest.position, hotspot.position) > growthDistance * 0.5f)
                {
                    return nearest;
                }
            }
        }

        // Priority 2: Expand toward last known player position
        if (expandTowardPlayer && playerDetected)
        {
            PatrolNode nearest = GetClosestNode(lastPlayerSpottedPos);
            if (nearest != null && Vector3.Distance(nearest.position, lastPlayerSpottedPos) > growthDistance * 0.5f)
            {
                return nearest;
            }
        }

        // Priority 3: Expand from frontier nodes (fewest connections)
        candidates = patrolNodes
            .Where(n => n.connections.Count < maxConnectionsPerNode)
            .OrderBy(n => n.connections.Count)
            .ThenByDescending(n => n.heat) // Prefer hot areas
            .Take(10)
            .ToList();

        if (candidates.Count > 0)
        {
            return candidates[Random.Range(0, candidates.Count)];
        }

        // Fallback: random node
        return patrolNodes[Random.Range(0, patrolNodes.Count)];
    }

    private Vector3 FindNewNodePosition(PatrolNode fromNode)
    {
        // Decide: grow along road or off-road?
        bool growAlongRoad = (Random.value < preferRoadGrowth) && fromNode.isOnRoad;

        Vector3 newPos = Vector3.zero;

        if (growAlongRoad)
        {
            // Find nearest road waypoints and grow along road
            var waypoints = cityGenerator.GetAllRoadWaypoints(30f);

            // Filter waypoints that are within growth distance
            var validWaypoints = waypoints
                .Where(w => Vector3.Distance(w, fromNode.position) > growthDistance * 0.3f
                         && Vector3.Distance(w, fromNode.position) < growthDistance * 1.5f)
                .Where(w => !IsTooCloseToExisting(w, growthDistance * 0.3f))
                .ToList();

            if (validWaypoints.Count > 0)
            {
                newPos = validWaypoints[Random.Range(0, validWaypoints.Count)];
            }
        }

        if (newPos == Vector3.zero)
        {
            // Off-road growth or fallback
            Vector3 direction;

            // Prefer growing toward player if detected
            if (playerDetected && Random.value < 0.6f)
            {
                direction = (lastPlayerSpottedPos - fromNode.position).normalized;
            }
            else
            {
                // Random direction with slight bias away from hive center (outward expansion)
                Vector3 awayFromHive = (fromNode.position - hiveCenter).normalized;
                Vector3 randomDir = Random.onUnitSphere;
                randomDir.y = 0;
                randomDir.Normalize();
                direction = (awayFromHive * 0.4f + randomDir * 0.6f).normalized;
            }

            newPos = fromNode.position + direction * growthDistance;
        }

        // Make sure position is valid
        if (IsTooCloseToExisting(newPos, growthDistance * 0.2f))
        {
            return Vector3.zero;
        }

        return newPos;
    }

    private PatrolNode.NodeType DetermineNodeType(Vector3 position, bool onRoad)
    {
        if (onRoad)
        {
            // Check if near intersection (Voronoi site)
            var nearestSite = cityGenerator.GetNearestSite(position);
            if (nearestSite != null && Vector3.Distance(position, nearestSite.position) < 15f)
            {
                return PatrolNode.NodeType.Intersection;
            }

            // Check if on main road
            foreach (var road in cityGenerator.roads)
            {
                if (road.ContainsPoint(position, 3f) && road.isMainRoad)
                {
                    return PatrolNode.NodeType.Junction;
                }
            }

            return PatrolNode.NodeType.RoadNode;
        }

        // Off-road node type detection
        Collider[] nearby = Physics.OverlapSphere(position, 15f, environmentLayer);

        if (nearby.Length >= 3)
        {
            return PatrolNode.NodeType.Cover;
        }
        else if (nearby.Length == 1 || nearby.Length == 2)
        {
            foreach (var col in nearby)
            {
                Renderer rend = col.GetComponentInChildren<Renderer>();
                if (rend && rend.bounds.size.y > 15f)
                {
                    return PatrolNode.NodeType.Vantage;
                }
            }
        }

        // Check for chokepoints
        int rayHits = 0;
        for (int i = 0; i < 8; i++)
        {
            float angle = i * 45f;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            if (Physics.Raycast(position, dir, 8f, environmentLayer))
            {
                rayHits++;
            }
        }

        if (rayHits >= 5)
        {
            return PatrolNode.NodeType.Chokepoint;
        }

        return PatrolNode.NodeType.Standard;
    }

    private void ConnectToNearbyNodes(PatrolNode node)
    {
        var nearby = patrolNodes
            .Where(n => n != node && n.connections.Count < maxConnectionsPerNode)
            .Where(n => Vector3.Distance(n.position, node.position) < connectionRadius)
            .OrderBy(n => Vector3.Distance(n.position, node.position))
            .Take(maxConnectionsPerNode - 1)
            .ToList();

        foreach (var nearNode in nearby)
        {
            // Prefer connecting same types (road to road, off-road to off-road)
            bool sameType = (node.isOnRoad && nearNode.isOnRoad) || (!node.isOnRoad && !nearNode.isOnRoad);
            if (!sameType && Random.value > 0.3f) continue;

            // Check line of sight
            if (!Physics.Linecast(node.position + Vector3.up, nearNode.position + Vector3.up, environmentLayer))
            {
                ConnectTwoNodes(node, nearNode);
            }
        }
    }

    private void SpawnInitialEnemies()
    {
        Debug.Log("Spawning initial AI units...");

        // Spawn tanks near hive center
        for (int i = 0; i < initialTankCount; i++)
        {
            PatrolNode spawnNode = patrolNodes[Random.Range(0, Mathf.Min(patrolNodes.Count, 3))];
            Vector3 offset = Random.insideUnitSphere * 10f;
            offset.y = 0;
            SpawnTank(spawnNode.position + offset, spawnNode);
        }

        // Spawn drones
        for (int i = 0; i < initialDroneCount; i++)
        {
            PatrolNode spawnNode = patrolNodes[Random.Range(0, patrolNodes.Count)];
            SpawnDrone(spawnNode.position + Vector3.up * 30f, spawnNode);
        }

        Debug.Log($"{enemies.Count} initial enemies deployed");
    }

    private void SpawnReinforcements()
    {
        if (enemies.Count >= maxEnemies || !playerDetected) return;

        // Spawn at frontier nodes (furthest from player but still in network)
        var frontierNodes = patrolNodes
            .OrderByDescending(n => Vector3.Distance(n.position, lastPlayerSpottedPos))
            .Take(5)
            .ToList();

        if (frontierNodes.Count == 0) return;

        PatrolNode spawnNode = frontierNodes[Random.Range(0, frontierNodes.Count)];

        int tanksToSpawn = Mathf.Min(currentDifficulty, maxEnemies - enemies.Count);
        int dronesToSpawn = Mathf.Min(currentDifficulty / 2, maxEnemies - enemies.Count - tanksToSpawn);

        for (int i = 0; i < tanksToSpawn; i++)
        {
            Vector3 offset = Random.insideUnitSphere * 10f;
            offset.y = 0;
            SpawnTank(spawnNode.position + offset, spawnNode);
        }

        for (int i = 0; i < dronesToSpawn; i++)
        {
            Vector3 offset = Random.insideUnitSphere * 15f;
            SpawnDrone(spawnNode.position + offset + Vector3.up * 35f, spawnNode);
        }

        Debug.Log($"Reinforcements: {tanksToSpawn} tanks, {dronesToSpawn} drones at frontier");
    }

    private void SpawnTank(Vector3 position, PatrolNode startNode)
    {
        GameObject tank = Instantiate(tankPrefab, position, Quaternion.identity);
        TankAI tankAI = tank.GetComponent<TankAI>();

        if (tankAI != null)
        {
            tankAI.Initialize(this, startNode, player);
        }

        enemies.Add(tank);
    }

    private void SpawnDrone(Vector3 position, PatrolNode startNode)
    {
        GameObject drone = Instantiate(dronePrefab, position, Quaternion.identity);
        DroneAI droneAI = drone.GetComponent<DroneAI>();

        if (droneAI != null)
        {
            droneAI.Initialize(this, startNode, player);
        }

        enemies.Add(drone);
    }

    private void IncreaseDifficulty()
    {
        currentDifficulty++;
        Debug.Log($"Difficulty increased to {currentDifficulty}");
    }

    // Helper methods
    private Vector3 SnapToTerrain(Vector3 pos)
    {
        if (terrain == null) return pos;

        TerrainData data = terrain.terrainData;
        Vector3 terrainPos = terrain.transform.position;

        float relX = (pos.x - terrainPos.x) / data.size.x;
        float relZ = (pos.z - terrainPos.z) / data.size.z;

        if (relX < 0 || relX > 1 || relZ < 0 || relZ > 1) return pos;

        float height = data.GetInterpolatedHeight(relX, relZ);
        return new Vector3(pos.x, terrainPos.y + height + 1f, pos.z);
    }

    private bool IsTooCloseToExisting(Vector3 pos, float minDist)
    {
        return patrolNodes.Any(n => Vector3.Distance(pos, n.position) < minDist);
    }

    private void ConnectTwoNodes(PatrolNode a, PatrolNode b)
    {
        if (!a.connections.Contains(b)) a.connections.Add(b);
        if (!b.connections.Contains(a)) b.connections.Add(a);
    }

    private void DecayHeat()
    {
        foreach (var node in patrolNodes)
        {
            node.heat = Mathf.Max(0, node.heat - heatDecayRate * Time.deltaTime);
        }
    }

    private void CleanupDestroyedEnemies()
    {
        enemies.RemoveAll(e => e == null);
    }

    // Public API
    public PatrolNode GetRandomNode()
    {
        if (patrolNodes.Count == 0) return null;
        return patrolNodes[Random.Range(0, patrolNodes.Count)];
    }

    public PatrolNode GetClosestNode(Vector3 pos, PatrolNode exclude = null)
    {
        PatrolNode closest = null;
        float minDist = Mathf.Infinity;

        foreach (var node in patrolNodes)
        {
            if (node == exclude) continue;
            float dist = Vector3.Distance(pos, node.position);
            if (dist < minDist)
            {
                minDist = dist;
                closest = node;
            }
        }

        return closest;
    }

    public PatrolNode GetClosestRoadNode(Vector3 pos)
    {
        return patrolNodes
            .Where(n => n.isOnRoad)
            .OrderBy(n => Vector3.Distance(n.position, pos))
            .FirstOrDefault();
    }

    public void OnPlayerSpotted(Vector3 spottedAt)
    {
        playerDetected = true;
        lastPlayerSpottedPos = spottedAt;
        lastPlayerSpottedTime = Time.time;

        // Add heat to nearby nodes
        foreach (var node in patrolNodes)
        {
            float dist = Vector3.Distance(node.position, spottedAt);
            if (dist < alertRadius)
            {
                node.heat = Mathf.Min(1f, node.heat + (1f - dist / alertRadius));
            }
        }

        if (enablePlayerLearning)
        {
            RecordPlayerVisit(spottedAt);
        }

        Debug.Log($"Player spotted at {spottedAt}! Network: {patrolNodes.Count} nodes, Enemies: {enemies.Count}");
    }

    // Player learning system
    private void TrackPlayerPosition()
    {
        if (player == null) return;

        Vector3 currentPos = player.position;
        if (Vector3.Distance(currentPos, lastTrackedPlayerPos) < 5f) return;

        lastTrackedPlayerPos = currentPos;
        RecordPlayerVisit(currentPos);
    }

    private void RecordPlayerVisit(Vector3 position)
    {
        PlayerVisit existing = playerVisitHistory
            .FirstOrDefault(v => Vector3.Distance(v.position, position) < visitFrequencyRadius);

        if (existing != null)
        {
            existing.visitCount++;
            existing.timestamp = Time.time;
        }
        else
        {
            playerVisitHistory.Add(new PlayerVisit(position, Time.time));
            if (playerVisitHistory.Count > maxTrackedLocations)
            {
                playerVisitHistory.RemoveAt(0);
            }
        }
    }

    private PlayerVisit GetMostVisitedLocation()
    {
        return playerVisitHistory
            .OrderByDescending(v => v.visitCount)
            .ThenByDescending(v => v.timestamp)
            .FirstOrDefault();
    }

    public int GetActiveEnemyCount() => enemies.Count(e => e != null);
    public int GetCurrentDifficulty() => currentDifficulty;
    public bool IsPlayerDetected() => playerDetected && (Time.time - lastPlayerSpottedTime) < 30f;
    public int GetNodeCount() => patrolNodes.Count;

    // Debug visualization
    void OnDrawGizmos()
    {
        if (!drawDebugGraph || patrolNodes == null || patrolNodes.Count == 0) return;

        // Draw hive center
        Gizmos.color = new Color(1, 0, 1, 0.5f);
        Gizmos.DrawWireSphere(hiveCenter, 20f);

        // Draw growth radius
        Gizmos.color = new Color(0, 1, 1, 0.1f);
        Gizmos.DrawWireSphere(hiveCenter, maxGrowthRadius);

        foreach (var node in patrolNodes)
        {
            // Color by type and age
            Color baseColor;

            if (node.isOnRoad)
            {
                baseColor = node.type == PatrolNode.NodeType.Intersection ? Color.magenta : Color.yellow;
            }
            else
            {
                switch (node.type)
                {
                    case PatrolNode.NodeType.Vantage:
                        baseColor = Color.cyan;
                        break;
                    case PatrolNode.NodeType.Chokepoint:
                        baseColor = Color.red;
                        break;
                    case PatrolNode.NodeType.Cover:
                        baseColor = Color.green;
                        break;
                    default:
                        baseColor = Color.gray;
                        break;
                }
            }

            // Show growth animation
            if (showGrowthAnimation && Time.time - node.creationTime < 2f)
            {
                float age = Time.time - node.creationTime;
                float pulse = Mathf.PingPong(age * 3f, 1f);
                baseColor = Color.Lerp(baseColor, Color.white, pulse);
            }

            // Apply heat
            if (drawHeatMap && node.heat > 0)
            {
                baseColor = Color.Lerp(baseColor, Color.red, node.heat);
            }

            Gizmos.color = baseColor;

            float size = node.isImportant ? 3f : (node.isOnRoad ? 2f : 1.5f);
            Gizmos.DrawSphere(node.position, size);

            // Draw connections
            Gizmos.color = node.isOnRoad ? new Color(1, 1, 0, 0.3f) : new Color(0.5f, 0.5f, 0.5f, 0.2f);
            foreach (var conn in node.connections)
            {
                if (conn != null)
                {
                    Gizmos.DrawLine(node.position, conn.position);
                }
            }
        }

        // Draw player alert zone
        if (playerDetected && Time.time - lastPlayerSpottedTime < 5f)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(lastPlayerSpottedPos, alertRadius);
        }

        // Draw player hotspots
        if (playerVisitHistory.Count > 0)
        {
            foreach (var visit in playerVisitHistory.Where(v => v.visitCount >= hotspotThreshold))
            {
                Gizmos.color = new Color(1, 0.5f, 0, 0.3f);
                Gizmos.DrawWireSphere(visit.position, visitFrequencyRadius);
            }
        }
    }

    void OnGUI()
    {
        if (!Application.isEditor) return;

        GUILayout.BeginArea(new Rect(10, 10, 300, 250));
        GUILayout.Box("=== HiveMind Status ===");
        GUILayout.Label($"Network Nodes: {patrolNodes.Count} / {maxNodes}");
        GUILayout.Label($"  - On Roads: {patrolNodes.Count(n => n.isOnRoad)}");
        GUILayout.Label($"  - Off Roads: {patrolNodes.Count(n => !n.isOnRoad)}");
        GUILayout.Label($"Active Enemies: {enemies.Count} / {maxEnemies}");
        GUILayout.Label($"Difficulty Level: {currentDifficulty}");
        GUILayout.Label($"Player Detected: {playerDetected}");
        GUILayout.Label($"Growth Range: {GetFurthestNodeDistance():F0}m / {maxGrowthRadius:F0}m");
        GUILayout.Label($"Next Growth: {Mathf.Max(0, nextGrowthTime - Time.time):F1}s");
        GUILayout.Label($"Player Hotspots: {playerVisitHistory.Count(v => v.visitCount >= hotspotThreshold)}");
        GUILayout.EndArea();
    }

    private float GetFurthestNodeDistance()
    {
        if (patrolNodes.Count == 0) return 0;
        return patrolNodes.Max(n => Vector3.Distance(n.position, hiveCenter));
    }
}