using UnityEngine;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// Advanced combat manager that coordinates all AI behavior, 
/// manages threat levels, and creates dynamic combat scenarios
/// </summary>
public class CombatManager : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private HiveMind hivemind;
    [SerializeField] private Transform player;

    [Header("Threat System")]
    [SerializeField] private float threatDecayRate = 0.05f;
    [SerializeField] private float maxThreatLevel = 100f;
    [SerializeField] private float detectionThreatGain = 10f;
    [SerializeField] private float combatThreatGain = 5f;

    [Header("Dynamic Responses")]
    [SerializeField] private bool enableDynamicSpawning = true;
    [SerializeField] private float highThreatThreshold = 60f;
    [SerializeField] private float emergencyResponseDelay = 10f;
    [SerializeField] private int emergencyUnitsToSpawn = 5;

    [Header("Behavior Modifiers")]
    [SerializeField] private bool enableAdaptiveBehavior = true;
    [SerializeField] private float aggressivenessMultiplier = 1f;
    [SerializeField] private float awarenessMultiplier = 1f;

    [Header("Audio & Atmosphere")]
    [SerializeField] private AudioSource combatMusicSource;
    [SerializeField] private AudioSource ambientMusicSource;
    [SerializeField] private float musicTransitionSpeed = 0.5f;
    [SerializeField] private AnimationCurve intensityCurve;

    // State tracking
    private float currentThreatLevel;
    private float timeSinceLastCombat;
    private float lastEmergencyResponseTime;
    private bool inCombat;
    private Vector3 lastCombatPosition;

    // Statistics
    private int totalPlayerDetections;
    private int totalEnemiesDefeated;
    private float longestSurvivalTime;
    private Dictionary<string, int> enemyTypeDefeats = new Dictionary<string, int>();

    void Start()
    {
        if (hivemind == null)
            hivemind = FindObjectOfType<HiveMind>();

        if (intensityCurve == null || intensityCurve.length == 0)
        {
            intensityCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);
        }
    }

    void Update()
    {
        UpdateThreatLevel();
        UpdateCombatState();
        UpdateAudioAtmosphere();
        CheckEmergencyResponse();
    }

    private void UpdateThreatLevel()
    {
        // Natural decay
        currentThreatLevel = Mathf.Max(0, currentThreatLevel - threatDecayRate * Time.deltaTime);

        // Time since last combat
        if (!inCombat)
        {
            timeSinceLastCombat += Time.deltaTime;
        }
        else
        {
            timeSinceLastCombat = 0f;
        }
    }

    private void UpdateCombatState()
    {
        bool wasInCombat = inCombat;

        // Check if player is currently engaged
        int nearbyEnemies = CountNearbyEnemies(player.position, 50f);
        inCombat = nearbyEnemies > 0 || currentThreatLevel > 20f;

        // Track combat transitions
        if (inCombat && !wasInCombat)
        {
            OnCombatStarted();
        }
        else if (!inCombat && wasInCombat)
        {
            OnCombatEnded();
        }
    }

    private void UpdateAudioAtmosphere()
    {
        if (combatMusicSource == null || ambientMusicSource == null) return;

        float intensity = currentThreatLevel / maxThreatLevel;
        float targetCombatVolume = intensityCurve.Evaluate(intensity);
        float targetAmbientVolume = 1f - targetCombatVolume;

        combatMusicSource.volume = Mathf.Lerp(combatMusicSource.volume, targetCombatVolume, musicTransitionSpeed * Time.deltaTime);
        ambientMusicSource.volume = Mathf.Lerp(ambientMusicSource.volume, targetAmbientVolume, musicTransitionSpeed * Time.deltaTime);
    }

    private void CheckEmergencyResponse()
    {
        if (!enableDynamicSpawning) return;

        if (currentThreatLevel > highThreatThreshold &&
            Time.time - lastEmergencyResponseTime > emergencyResponseDelay)
        {
            TriggerEmergencyResponse();
            lastEmergencyResponseTime = Time.time;
        }
    }

    private void TriggerEmergencyResponse()
    {
        Debug.Log("EMERGENCY RESPONSE ACTIVATED! High threat detected.");

        // Notify hivemind to send reinforcements
        if (hivemind != null)
        {
            hivemind.OnPlayerSpotted(player.position);
        }

        // Could spawn additional units here
        // SpawnEmergencyUnits();
    }

    private int CountNearbyEnemies(Vector3 position, float radius)
    {
        int count = 0;
        Collider[] colliders = Physics.OverlapSphere(position, radius);

        foreach (var col in colliders)
        {
            if (col.GetComponent<TankAI>() != null || col.GetComponent<DroneAI>() != null)
            {
                count++;
            }
        }

        return count;
    }

    // Public API - Called by AI units
    public void OnPlayerDetected(Vector3 position)
    {
        currentThreatLevel = Mathf.Min(maxThreatLevel, currentThreatLevel + detectionThreatGain);
        lastCombatPosition = position;
        totalPlayerDetections++;

        if (hivemind != null)
        {
            hivemind.OnPlayerSpotted(position);
        }
    }

    public void OnEnemyDestroyed(GameObject enemy)
    {
        totalEnemiesDefeated++;

        string enemyType = enemy.GetComponent<TankAI>() != null ? "Tank" : "Drone";

        if (!enemyTypeDefeats.ContainsKey(enemyType))
            enemyTypeDefeats[enemyType] = 0;

        enemyTypeDefeats[enemyType]++;

        // Increase threat when enemies are destroyed
        currentThreatLevel = Mathf.Min(maxThreatLevel, currentThreatLevel + combatThreatGain);
    }

    public void OnPlayerFired(Vector3 position)
    {
        currentThreatLevel = Mathf.Min(maxThreatLevel, currentThreatLevel + combatThreatGain * 0.5f);
        lastCombatPosition = position;

        // Alert nearby AI
        AlertNearbyEnemies(position, 30f);
    }

    private void AlertNearbyEnemies(Vector3 position, float radius)
    {
        Collider[] colliders = Physics.OverlapSphere(position, radius);

        foreach (var col in colliders)
        {
            TankAI tank = col.GetComponent<TankAI>();
            DroneAI drone = col.GetComponent<DroneAI>();

            // They will handle the alert in their own systems
            if (tank != null || drone != null)
            {
                OnPlayerDetected(position);
            }
        }
    }

    private void OnCombatStarted()
    {
        Debug.Log("Combat initiated! Current threat level: " + currentThreatLevel);
    }

    private void OnCombatEnded()
    {
        float combatDuration = Time.time - (Time.time - timeSinceLastCombat);
        longestSurvivalTime = Mathf.Max(longestSurvivalTime, combatDuration);

        Debug.Log($"Combat ended. Duration: {combatDuration:F1}s, Enemies defeated: {totalEnemiesDefeated}");
    }

    // Getters for AI behavior modifiers
    public float GetAggressivenessModifier()
    {
        return aggressivenessMultiplier * (1f + currentThreatLevel / maxThreatLevel);
    }

    public float GetAwarenessModifier()
    {
        return awarenessMultiplier * (1f + currentThreatLevel / (maxThreatLevel * 2f));
    }

    public bool ShouldUseAggressiveTactics()
    {
        return currentThreatLevel > highThreatThreshold * 0.7f;
    }

    public float GetCurrentThreatLevel() => currentThreatLevel;
    public bool IsInCombat() => inCombat;

    // Debug visualization
    void OnGUI()
    {
        if (!Application.isEditor) return;

        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Box("Combat Manager Stats");
        GUILayout.Label($"Threat Level: {currentThreatLevel:F1} / {maxThreatLevel}");
        GUILayout.Label($"In Combat: {inCombat}");
        GUILayout.Label($"Time Since Combat: {timeSinceLastCombat:F1}s");
        GUILayout.Label($"Total Detections: {totalPlayerDetections}");
        GUILayout.Label($"Enemies Defeated: {totalEnemiesDefeated}");

        if (hivemind != null)
        {
            GUILayout.Label($"Active Enemies: {hivemind.GetActiveEnemyCount()}");
            GUILayout.Label($"Difficulty Level: {hivemind.GetCurrentDifficulty()}");
        }

        GUILayout.EndArea();
    }
}