using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
public class DroneAI : MonoBehaviour
{
    [Header("Core References")]
    public HiveMind hivemind;
    public Transform player;

    [Header("Movement Settings")]
    [SerializeField] private float cruiseSpeed = 15f;
    [SerializeField] private float chaseSpeed = 25f;
    [SerializeField] private float acceleration = 8f;
    [SerializeField] private float turnSpeed = 5f;
    [SerializeField] private float tiltAmount = 25f;

    [Header("Flight Behavior")]
    [SerializeField] private float minHeight = 8f;
    [SerializeField] private float maxHeight = 50f;
    [SerializeField] private float heightVariation = 5f;
    [SerializeField] private float bobSpeed = 1f;
    [SerializeField] private float bobAmount = 2f;

    [Header("Obstacle Avoidance")]
    [SerializeField] private float visionDistance = 20f;
    [SerializeField] private float avoidanceStrength = 5f;
    [SerializeField] private LayerMask obstacleLayer = -1;
    [SerializeField] private int raycastCount = 12;

    [Header("Player Detection")]
    [SerializeField] private float detectionRadius = 40f;
    [SerializeField] private float attackRange = 30f;
    [SerializeField] private float loseTargetDistance = 60f;
    [SerializeField] private float fieldOfView = 120f;
    [SerializeField] private LayerMask playerLayer;

    [Header("Formation Flying")]
    [SerializeField] private float separationRadius = 8f;
    [SerializeField] private float separationStrength = 3f;
    [SerializeField] private float alignmentStrength = 1f;
    [SerializeField] private float cohesionStrength = 0.5f;

    [Header("Visual Effects")]
    [SerializeField] private Light spotLight;

    // State
    private enum DroneState { Patrol, Investigate, Chase, Attack, Return }
    private DroneState currentState = DroneState.Patrol;
    private HiveMind.PatrolNode targetNode;
    private HiveMind.PatrolNode homeNode;
    private Vector3 lastKnownPlayerPos;
    private float stateTimer;
    private float alertLevel;
    private float targetHeight;

    // Components
    private Rigidbody rb;
    private Vector3 velocity;
    private List<DroneAI> nearbyDrones = new List<DroneAI>();

    // Cached calculations
    private Vector3 desiredVelocity;
    private Vector3 avoidanceForce;
    private Vector3 flockingForce;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.None;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        targetHeight = Random.Range(minHeight + heightVariation, maxHeight - heightVariation);
    }

    void Start()
    {
        if (hivemind != null && targetNode == null)
        {
            targetNode = hivemind.GetRandomNode();
            homeNode = targetNode;
        }

        InvokeRepeating(nameof(UpdateNearbyDrones), 0f, 0.5f);
        InvokeRepeating(nameof(ScanForPlayer), 0.1f, 0.2f);
    }

    void FixedUpdate()
    {
        UpdateState();
        CalculateMovement();
        ApplyMovement();
    }

    private void UpdateState()
    {
        stateTimer += Time.fixedDeltaTime;
        alertLevel = Mathf.Max(0, alertLevel - Time.fixedDeltaTime * 0.5f);

        switch (currentState)
        {
            case DroneState.Patrol:
                HandlePatrolState();
                break;

            case DroneState.Investigate:
                HandleInvestigateState();
                break;

            case DroneState.Chase:
                HandleChaseState();
                break;

            case DroneState.Attack:
                HandleAttackState();
                break;

            case DroneState.Return:
                HandleReturnState();
                break;
        }
    }

    private void HandlePatrolState()
    {
        if (targetNode == null)
        {
            targetNode = hivemind.GetRandomNode();
            return;
        }

        if (Vector3.Distance(transform.position, targetNode.position) < 5f)
        {
            // Choose next node intelligently
            if (targetNode.connections != null && targetNode.connections.Count > 0)
            {
                targetNode = targetNode.connections[Random.Range(0, targetNode.connections.Count)];
            }
            else
            {
                targetNode = hivemind.GetRandomNode();
            }

            targetHeight = Random.Range(minHeight + heightVariation, maxHeight - heightVariation);
        }
    }

    private void HandleInvestigateState()
    {
        if (Vector3.Distance(transform.position, lastKnownPlayerPos) < 8f || stateTimer > 10f)
        {
            if (CanSeePlayer())
            {
                TransitionToState(DroneState.Chase);
            }
            else
            {
                TransitionToState(DroneState.Return);
            }
        }
    }

    private void HandleChaseState()
    {
        if (!CanSeePlayer() && Vector3.Distance(transform.position, player.position) > loseTargetDistance)
        {
            lastKnownPlayerPos = player.position;
            TransitionToState(DroneState.Investigate);
        }
        else if (Vector3.Distance(transform.position, player.position) < attackRange)
        {
            TransitionToState(DroneState.Attack);
        }
        else
        {
            lastKnownPlayerPos = player.position;
        }
    }

    private void HandleAttackState()
    {
        float distToPlayer = Vector3.Distance(transform.position, player.position);

        if (distToPlayer > attackRange * 1.5f)
        {
            TransitionToState(DroneState.Chase);
        }

        // Circle strafe behavior
        Vector3 toPlayer = player.position - transform.position;
        Vector3 perpendicular = Vector3.Cross(toPlayer.normalized, Vector3.up);
        desiredVelocity += perpendicular * cruiseSpeed * 0.5f;
    }

    private void HandleReturnState()
    {
        if (homeNode != null && Vector3.Distance(transform.position, homeNode.position) < 5f)
        {
            targetNode = homeNode;
            TransitionToState(DroneState.Patrol);
        }
    }

    private void CalculateMovement()
    {
        // Base desired velocity
        Vector3 targetPos = GetTargetPosition();
        Vector3 direction = (targetPos - transform.position).normalized;
        float currentSpeed = GetCurrentSpeed();

        desiredVelocity = direction * currentSpeed;

        // Add height control
        float heightError = targetHeight - transform.position.y;
        desiredVelocity.y += heightError * 2f;

        // Add subtle bobbing
        desiredVelocity.y += Mathf.Sin(Time.time * bobSpeed) * bobAmount;

        // Calculate avoidance
        avoidanceForce = CalculateAvoidance();

        // Calculate flocking with nearby drones
        flockingForce = CalculateFlocking();

        // Combine forces
        velocity = desiredVelocity + avoidanceForce * avoidanceStrength + flockingForce;

        // Height constraints
        if (transform.position.y < minHeight)
        {
            velocity.y = Mathf.Max(velocity.y, (minHeight - transform.position.y) * 5f);
        }
        else if (transform.position.y > maxHeight)
        {
            velocity.y = Mathf.Min(velocity.y, (maxHeight - transform.position.y) * 5f);
        }
    }

    private Vector3 CalculateAvoidance()
    {
        Vector3 avoid = Vector3.zero;
        float angleStep = 360f / raycastCount;

        for (int i = 0; i < raycastCount; i++)
        {
            float angle = angleStep * i;

            // Create rays in a sphere pattern around the drone
            for (int elevation = -30; elevation <= 30; elevation += 30)
            {
                Quaternion rotation = Quaternion.Euler(elevation, angle, 0);
                Vector3 rayDir = rotation * Vector3.forward;

                if (Physics.Raycast(transform.position, rayDir, out RaycastHit hit, visionDistance, obstacleLayer))
                {
                    float weight = 1f - (hit.distance / visionDistance);
                    avoid += (transform.position - hit.point).normalized * weight;
                }
            }
        }

        return avoid;
    }

    private Vector3 CalculateFlocking()
    {
        if (nearbyDrones.Count == 0) return Vector3.zero;

        Vector3 separation = Vector3.zero;
        Vector3 alignment = Vector3.zero;
        Vector3 cohesion = Vector3.zero;
        int count = 0;

        foreach (DroneAI other in nearbyDrones)
        {
            if (other == null || other == this) continue;

            float dist = Vector3.Distance(transform.position, other.transform.position);

            // Separation
            if (dist < separationRadius && dist > 0)
            {
                separation += (transform.position - other.transform.position).normalized / dist;
                count++;
            }

            // Alignment
            alignment += other.rb.linearVelocity;

            // Cohesion
            cohesion += other.transform.position;
        }

        if (count > 0)
        {
            alignment /= nearbyDrones.Count;
            cohesion /= nearbyDrones.Count;
            cohesion = (cohesion - transform.position).normalized;

            return separation * separationStrength +
                   alignment.normalized * alignmentStrength +
                   cohesion * cohesionStrength;
        }

        return Vector3.zero;
    }

    private void ApplyMovement()
    {
        // Smooth acceleration
        rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, velocity, acceleration * Time.fixedDeltaTime);

        // Rotation with banking
        if (rb.linearVelocity.magnitude > 0.1f)
        {
            // Look rotation
            Quaternion targetRot = Quaternion.LookRotation(rb.linearVelocity.normalized);

            // Add model offset (forward is -X)
            targetRot *= Quaternion.Euler(0, -90f, 0);

            // Calculate banking based on turn rate
            Vector3 angularVel = rb.angularVelocity;
            float bank = -angularVel.y * tiltAmount;
            targetRot *= Quaternion.Euler(0, 0, bank);

            rb.rotation = Quaternion.Slerp(rb.rotation, targetRot, turnSpeed * Time.fixedDeltaTime);
        }
    }

    private void ScanForPlayer()
    {
        if (player == null || currentState == DroneState.Return) return;

        float distToPlayer = Vector3.Distance(transform.position, player.position);

        if (distToPlayer < detectionRadius)
        {
            Vector3 dirToPlayer = (player.position - transform.position).normalized;
            float angle = Vector3.Angle(transform.forward, dirToPlayer);

            if (angle < fieldOfView * 0.5f)
            {
                if (Physics.Raycast(transform.position, dirToPlayer, out RaycastHit hit, detectionRadius))
                {
                    if (hit.transform == player || hit.transform.IsChildOf(player))
                    {
                        OnPlayerDetected();
                    }
                }
            }
        }
    }

    private void OnPlayerDetected()
    {
        if (currentState == DroneState.Patrol || currentState == DroneState.Investigate)
        {
            hivemind?.OnPlayerSpotted(player.position);
            lastKnownPlayerPos = player.position;
            TransitionToState(DroneState.Chase);
            alertLevel = 1f;
        }
    }

    private bool CanSeePlayer()
    {
        if (player == null) return false;

        Vector3 dirToPlayer = (player.position - transform.position).normalized;
        float distToPlayer = Vector3.Distance(transform.position, player.position);

        if (Physics.Raycast(transform.position, dirToPlayer, out RaycastHit hit, distToPlayer + 2f))
        {
            return hit.transform == player || hit.transform.IsChildOf(player);
        }

        return false;
    }

    private void UpdateNearbyDrones()
    {
        nearbyDrones.Clear();
        Collider[] nearbyColliders = Physics.OverlapSphere(transform.position, separationRadius * 2f);

        foreach (Collider col in nearbyColliders)
        {
            DroneAI drone = col.GetComponent<DroneAI>();
            if (drone != null && drone != this)
            {
                nearbyDrones.Add(drone);
            }
        }
    }

    private Vector3 GetTargetPosition()
    {
        switch (currentState)
        {
            case DroneState.Patrol:
                return targetNode != null ? targetNode.position : transform.position;

            case DroneState.Investigate:
                return lastKnownPlayerPos;

            case DroneState.Chase:
            case DroneState.Attack:
                return player != null ? player.position : lastKnownPlayerPos;

            case DroneState.Return:
                return homeNode != null ? homeNode.position : transform.position;

            default:
                return transform.position;
        }
    }

    private float GetCurrentSpeed()
    {
        switch (currentState)
        {
            case DroneState.Chase:
            case DroneState.Attack:
                return chaseSpeed;
            default:
                return cruiseSpeed;
        }
    }

    private void TransitionToState(DroneState newState)
    {
        currentState = newState;
        stateTimer = 0f;

        if (newState == DroneState.Return && homeNode == null)
        {
            homeNode = hivemind.GetClosestNode(transform.position);
        }
    }

    public void Initialize(HiveMind hive, HiveMind.PatrolNode startNode, Transform playerTransform)
    {
        hivemind = hive;
        targetNode = startNode;
        homeNode = startNode;
        player = playerTransform;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, detectionRadius);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, attackRange);

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, separationRadius);
    }
}