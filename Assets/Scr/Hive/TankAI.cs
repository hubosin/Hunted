using UnityEngine;
using System.Collections.Generic;
using System.Collections;

[RequireComponent(typeof(Rigidbody))]
public class TankAI : MonoBehaviour
{
    [Header("Core References")]
    public HiveMind hivemind;
    public Transform player;
    public Transform turret;
    public Transform barrelEnd;

    [Header("Movement Settings")]
    [SerializeField] private float cruiseSpeed = 8f;
    [SerializeField] private float chaseSpeed = 12f;
    [SerializeField] private float acceleration = 5f;
    [SerializeField] private float turnSpeed = 3f;
    [SerializeField] private float reverseSpeed = 4f;

    [Header("Stability")]
    [SerializeField] private float uprightForce = 15f;
    [SerializeField] private float groundCheckDistance = 2f;
    [SerializeField] private LayerMask groundLayer = -1;
    [SerializeField] private float maxSlopeAngle = 45f;

    [Header("Obstacle Avoidance")]
    [SerializeField] private float visionDistance = 15f;
    [SerializeField] private float avoidanceStrength = 3f;
    [SerializeField] private float stuckThreshold = 0.5f;
    [SerializeField] private float stuckTime = 2f;
    [SerializeField] private int raycastCount = 5;

    [Header("Combat")]
    [SerializeField] private float detectionRadius = 50f;
    [SerializeField] private float attackRange = 40f;
    [SerializeField] private float optimalRange = 30f;
    [SerializeField] private float fieldOfView = 150f;
    [SerializeField] private float turretTurnSpeed = 2f;
    [SerializeField] private float fireRate = 2f;
    [SerializeField] private float weaponDamage = 25f;
    [SerializeField] private float maxWeaponRange = 100f;
    [SerializeField] private LayerMask weaponHitLayers = -1;

    [Header("Tactical Behavior")]
    [SerializeField] private float coverSeekRadius = 20f;
    [SerializeField] private float retreatHealthThreshold = 0.3f;
    [SerializeField] private bool useSquadTactics = true;

    [Header("Visual Effects")]
    [SerializeField] private Light searchLight;
    [SerializeField] private LineRenderer weaponTracer;
    [SerializeField] private float tracerDuration = 0.1f;

    // State
    private enum TankState { Patrol, Investigate, Chase, Attack, Retreat, Reposition }
    private TankState currentState = TankState.Patrol;
    private HiveMind.PatrolNode targetNode;
    private HiveMind.PatrolNode homeNode;
    private Vector3 lastKnownPlayerPos;
    private Vector3 coverPosition;
    private float stateTimer;
    private float alertLevel;
    private float nextFireTime;
    private bool isStuck;
    private float stuckTimer;
    private Vector3 lastPosition;

    // Components & cache
    private Rigidbody rb;
    private float currentHealth = 1f;
    private List<TankAI> squadMates = new List<TankAI>();
    private bool isGrounded;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.mass = 1000f;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
    }

    void Start()
    {
        if (hivemind != null && targetNode == null)
        {
            targetNode = hivemind.GetRandomNode();
            homeNode = targetNode;
        }

        lastPosition = transform.position;

        InvokeRepeating(nameof(CheckIfStuck), 1f, 1f);
        InvokeRepeating(nameof(ScanForPlayer), 0.2f, 0.3f);
        InvokeRepeating(nameof(UpdateSquad), 0f, 1f);
    }

    void FixedUpdate()
    {
        CheckGrounded();
        UpdateState();
        CalculateMovement();
        UpdateTurret();
        MaintainStability();
        UpdateVisuals();
    }

    private void CheckGrounded()
    {
        isGrounded = Physics.Raycast(transform.position, Vector3.down, groundCheckDistance, groundLayer);
    }

    private void UpdateState()
    {
        stateTimer += Time.fixedDeltaTime;
        alertLevel = Mathf.Max(0, alertLevel - Time.fixedDeltaTime * 0.3f);

        switch (currentState)
        {
            case TankState.Patrol:
                HandlePatrolState();
                break;

            case TankState.Investigate:
                HandleInvestigateState();
                break;

            case TankState.Chase:
                HandleChaseState();
                break;

            case TankState.Attack:
                HandleAttackState();
                break;

            case TankState.Retreat:
                HandleRetreatState();
                break;

            case TankState.Reposition:
                HandleRepositionState();
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

        float arrivalDist = 5f;
        if (Vector3.Distance(transform.position, targetNode.position) < arrivalDist)
        {
            // Intelligent pathfinding through connected nodes
            if (targetNode.connections != null && targetNode.connections.Count > 0)
            {
                // Prefer important nodes
                var importantNodes = targetNode.connections.FindAll(n => n.isImportant);
                if (importantNodes.Count > 0 && Random.value > 0.7f)
                {
                    targetNode = importantNodes[Random.Range(0, importantNodes.Count)];
                }
                else
                {
                    targetNode = targetNode.connections[Random.Range(0, targetNode.connections.Count)];
                }
            }
            else
            {
                targetNode = hivemind.GetRandomNode();
            }
        }
    }

    private void HandleInvestigateState()
    {
        if (Vector3.Distance(transform.position, lastKnownPlayerPos) < 10f || stateTimer > 15f)
        {
            if (CanSeePlayer())
            {
                TransitionToState(TankState.Chase);
            }
            else
            {
                // Search nearby area
                if (stateTimer > 8f)
                {
                    targetNode = hivemind.GetClosestNode(transform.position);
                    TransitionToState(TankState.Patrol);
                }
            }
        }
    }

    private void HandleChaseState()
    {
        if (!CanSeePlayer() && Vector3.Distance(transform.position, player.position) > detectionRadius * 1.5f)
        {
            lastKnownPlayerPos = player.position;
            TransitionToState(TankState.Investigate);
        }
        else
        {
            float distToPlayer = Vector3.Distance(transform.position, player.position);
            lastKnownPlayerPos = player.position;

            if (distToPlayer < attackRange)
            {
                TransitionToState(TankState.Attack);
            }
        }
    }

    private void HandleAttackState()
    {
        float distToPlayer = Vector3.Distance(transform.position, player.position);

        // Health-based retreat
        if (currentHealth < retreatHealthThreshold)
        {
            FindCover();
            TransitionToState(TankState.Retreat);
            return;
        }

        // Range management
        if (distToPlayer > attackRange * 1.5f)
        {
            TransitionToState(TankState.Chase);
        }
        else if (distToPlayer < optimalRange * 0.5f)
        {
            // Too close, back up while firing
            TransitionToState(TankState.Reposition);
        }

        // Attempt to fire
        TryFireWeapon();
    }

    private void HandleRetreatState()
    {
        if (Vector3.Distance(transform.position, coverPosition) < 5f || stateTimer > 10f)
        {
            if (currentHealth > retreatHealthThreshold + 0.2f)
            {
                TransitionToState(TankState.Attack);
            }
        }
    }

    private void HandleRepositionState()
    {
        float distToPlayer = Vector3.Distance(transform.position, player.position);

        if (distToPlayer > optimalRange)
        {
            TransitionToState(TankState.Attack);
        }
        else if (stateTimer > 3f)
        {
            TransitionToState(TankState.Attack);
        }
    }

    private void CalculateMovement()
    {
        Vector3 targetPos = GetTargetPosition();
        Vector3 direction = (targetPos - transform.position);
        direction.y = 0;

        float distance = direction.magnitude;
        direction = direction.normalized;

        // Determine if we need to reverse
        float forwardDot = Vector3.Dot(transform.forward, direction);
        bool shouldReverse = (currentState == TankState.Reposition && forwardDot > 0.5f);

        // Speed calculation
        float targetSpeed = GetCurrentSpeed();
        if (shouldReverse) targetSpeed = -reverseSpeed;

        // Obstacle avoidance
        Vector3 avoidance = CalculateAvoidance();
        Vector3 finalDirection = (direction + avoidance * avoidanceStrength).normalized;

        // Apply movement
        Vector3 targetVelocity = finalDirection * targetSpeed;
        targetVelocity.y = rb.linearVelocity.y; // Preserve vertical velocity

        rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, targetVelocity, acceleration * Time.fixedDeltaTime);

        // Rotation
        if (!shouldReverse && finalDirection != Vector3.zero)
        {
            Quaternion targetRot = Quaternion.LookRotation(finalDirection);
            targetRot *= Quaternion.Euler(0, -90f, 0); // Model offset

            rb.rotation = Quaternion.Slerp(rb.rotation, targetRot, turnSpeed * Time.fixedDeltaTime);
        }
    }

    private Vector3 CalculateAvoidance()
    {
        Vector3 avoid = Vector3.zero;
        Vector3[] checkDirections = new Vector3[raycastCount];

        // Generate forward-focused rays
        for (int i = 0; i < raycastCount; i++)
        {
            float angle = -60f + (120f / (raycastCount - 1)) * i;
            checkDirections[i] = Quaternion.Euler(0, angle, 0) * transform.forward;
        }

        foreach (var dir in checkDirections)
        {
            Vector3 rayStart = transform.position + Vector3.up * 1f;

            if (Physics.Raycast(rayStart, dir, out RaycastHit hit, visionDistance))
            {
                if (hit.collider.gameObject != player.gameObject)
                {
                    float weight = 1f - (hit.distance / visionDistance);
                    avoid += (transform.position - hit.point).normalized * weight * weight;
                }
            }
        }

        return avoid;
    }

    private void UpdateTurret()
    {
        if (turret == null) return;

        Vector3 targetLookPos = currentState == TankState.Attack || currentState == TankState.Reposition
            ? player.position
            : transform.position + transform.forward * 10f;

        Vector3 dirToTarget = (targetLookPos - turret.position).normalized;
        dirToTarget.y = 0;

        if (dirToTarget != Vector3.zero)
        {
            Quaternion targetTurretRot = Quaternion.LookRotation(dirToTarget);
            turret.rotation = Quaternion.Slerp(turret.rotation, targetTurretRot, turretTurnSpeed * Time.fixedDeltaTime);
        }
    }

    private void TryFireWeapon()
    {
        if (Time.time < nextFireTime || barrelEnd == null) return;

        // Check if turret is aimed at target
        Vector3 toPlayer = (player.position - turret.position).normalized;
        float aimAngle = Vector3.Angle(turret.forward, toPlayer);

        if (aimAngle < 5f && CanSeePlayer())
        {
            FireRaycast();
            nextFireTime = Time.time + (1f / fireRate);
        }
    }

    private void FireRaycast()
    {
        Vector3 fireDirection = turret.forward;
        Vector3 fireOrigin = barrelEnd.position;

        // Perform raycast
        if (Physics.Raycast(fireOrigin, fireDirection, out RaycastHit hit, maxWeaponRange, weaponHitLayers))
        {
            // Check if we hit the player
            if (hit.transform == player || hit.transform.IsChildOf(player))
            {
                // Apply damage to player here
                // Example: hit.transform.GetComponent<PlayerHealth>()?.TakeDamage(weaponDamage);
                Debug.Log($"Tank hit player for {weaponDamage} damage at {hit.point}");
            }

            // Visual tracer to hit point
            if (weaponTracer != null)
            {
                StartCoroutine(ShowWeaponTracer(fireOrigin, hit.point));
            }

            // You can add impact effects at hit.point here
        }
        else
        {
            // Shot missed, show tracer to max range
            if (weaponTracer != null)
            {
                Vector3 endPoint = fireOrigin + fireDirection * maxWeaponRange;
                StartCoroutine(ShowWeaponTracer(fireOrigin, endPoint));
            }
        }

        // Apply recoil
        rb.AddForce(-turret.forward * 500f, ForceMode.Impulse);
    }

    private System.Collections.IEnumerator ShowWeaponTracer(Vector3 start, Vector3 end)
    {
        weaponTracer.enabled = true;
        weaponTracer.SetPosition(0, start);
        weaponTracer.SetPosition(1, end);

        yield return new WaitForSeconds(tracerDuration);

        weaponTracer.enabled = false;
    }

    private void MaintainStability()
    {
        // Keep tank upright
        Vector3 uprightTorque = Vector3.Cross(transform.up, Vector3.up);
        rb.AddTorque(uprightTorque * uprightForce, ForceMode.Acceleration);

        // Check slope angle
        if (Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, groundCheckDistance))
        {
            float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);

            if (slopeAngle > maxSlopeAngle)
            {
                // Find alternative route
                Vector3 awayFromSlope = Vector3.ProjectOnPlane(-hit.normal, Vector3.up).normalized;
                rb.AddForce(awayFromSlope * 10f, ForceMode.Acceleration);
            }
        }
    }

    private void ScanForPlayer()
    {
        if (player == null || currentState == TankState.Retreat) return;

        float distToPlayer = Vector3.Distance(transform.position, player.position);

        if (distToPlayer < detectionRadius)
        {
            Vector3 dirToPlayer = (player.position - transform.position).normalized;
            float angle = Vector3.Angle(transform.forward, dirToPlayer);

            if (angle < fieldOfView * 0.5f)
            {
                if (Physics.Raycast(transform.position + Vector3.up * 2f, dirToPlayer, out RaycastHit hit, detectionRadius))
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
        if (currentState == TankState.Patrol || currentState == TankState.Investigate)
        {
            hivemind?.OnPlayerSpotted(player.position);
            lastKnownPlayerPos = player.position;
            TransitionToState(TankState.Chase);
            alertLevel = 1f;

            // Alert nearby squad
            AlertSquad();
        }
    }

    private bool CanSeePlayer()
    {
        if (player == null) return false;

        Vector3 dirToPlayer = (player.position - transform.position).normalized;
        float distToPlayer = Vector3.Distance(transform.position, player.position);

        if (Physics.Raycast(transform.position + Vector3.up * 2f, dirToPlayer, out RaycastHit hit, distToPlayer + 2f))
        {
            return hit.transform == player || hit.transform.IsChildOf(player);
        }

        return false;
    }

    private void CheckIfStuck()
    {
        float distanceMoved = Vector3.Distance(transform.position, lastPosition);
        lastPosition = transform.position;

        if (rb.linearVelocity.magnitude > 0.5f && distanceMoved < stuckThreshold)
        {
            stuckTimer += 1f;

            if (stuckTimer > stuckTime)
            {
                isStuck = true;
                UnstuckManeuver();
            }
        }
        else
        {
            stuckTimer = 0f;
            isStuck = false;
        }
    }

    private void UnstuckManeuver()
    {
        // Reverse and turn
        rb.AddForce(-transform.forward * 5000f, ForceMode.Impulse);
        rb.AddTorque(Vector3.up * Random.Range(-500f, 500f), ForceMode.Impulse);

        // Find new path
        if (currentState == TankState.Patrol && hivemind != null)
        {
            targetNode = hivemind.GetRandomNode();
        }

        stuckTimer = 0f;
    }

    private void FindCover()
    {
        // Simple cover finding - move away from player
        Vector3 awayFromPlayer = (transform.position - player.position).normalized;
        coverPosition = transform.position + awayFromPlayer * coverSeekRadius;

        // Try to find actual cover object
        Collider[] nearbyObjects = Physics.OverlapSphere(transform.position, coverSeekRadius);
        float bestCoverScore = 0f;

        foreach (Collider col in nearbyObjects)
        {
            if (col.gameObject == gameObject || col.transform == player) continue;

            Vector3 toObject = col.transform.position - transform.position;
            float distToObject = toObject.magnitude;
            Vector3 objectToPlayer = player.position - col.transform.position;

            // Good cover is between us and player
            if (Vector3.Dot(toObject.normalized, awayFromPlayer) > 0)
            {
                float coverScore = 1f / (distToObject + 1f);
                if (coverScore > bestCoverScore)
                {
                    bestCoverScore = coverScore;
                    coverPosition = col.transform.position;
                }
            }
        }
    }

    private void UpdateSquad()
    {
        if (!useSquadTactics) return;

        squadMates.Clear();
        Collider[] nearbyColliders = Physics.OverlapSphere(transform.position, 30f);

        foreach (Collider col in nearbyColliders)
        {
            TankAI tank = col.GetComponent<TankAI>();
            if (tank != null && tank != this)
            {
                squadMates.Add(tank);
            }
        }
    }

    private void AlertSquad()
    {
        foreach (TankAI mate in squadMates)
        {
            if (mate != null && mate.currentState == TankState.Patrol)
            {
                mate.lastKnownPlayerPos = lastKnownPlayerPos;
                mate.TransitionToState(TankState.Investigate);
            }
        }
    }

    private void UpdateVisuals()
    {
        // Search light
        if (searchLight != null)
        {
            searchLight.color = currentState == TankState.Attack ? Color.red :
                                currentState == TankState.Chase ? Color.yellow : Color.white;
            searchLight.intensity = Mathf.Lerp(1f, 3f, alertLevel);
        }
    }

    private Vector3 GetTargetPosition()
    {
        switch (currentState)
        {
            case TankState.Patrol:
                return targetNode != null ? targetNode.position : transform.position;

            case TankState.Investigate:
                return lastKnownPlayerPos;

            case TankState.Chase:
                return player != null ? player.position : lastKnownPlayerPos;

            case TankState.Attack:
                // Maintain optimal distance
                Vector3 toPlayer = player.position - transform.position;
                float dist = toPlayer.magnitude;
                if (dist < optimalRange)
                {
                    return transform.position - toPlayer.normalized * 5f;
                }
                return player.position;

            case TankState.Retreat:
                return coverPosition;

            case TankState.Reposition:
                // Back away from player
                Vector3 away = (transform.position - player.position).normalized;
                return transform.position + away * 10f;

            default:
                return transform.position;
        }
    }

    private float GetCurrentSpeed()
    {
        switch (currentState)
        {
            case TankState.Chase:
            case TankState.Retreat:
                return chaseSpeed;
            case TankState.Attack:
                return cruiseSpeed * 0.7f; // Slower when attacking
            default:
                return cruiseSpeed;
        }
    }

    private void TransitionToState(TankState newState)
    {
        currentState = newState;
        stateTimer = 0f;
    }

    public void TakeDamage(float damage)
    {
        currentHealth -= damage;

        if (currentHealth <= 0)
        {
            Die();
        }
        else if (currentHealth < retreatHealthThreshold && currentState == TankState.Attack)
        {
            FindCover();
            TransitionToState(TankState.Retreat);
        }
    }

    private void Die()
    {
        // Add explosion effect here
        Destroy(gameObject);
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
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(transform.position, detectionRadius);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, attackRange);

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, optimalRange);

        if (currentState == TankState.Retreat)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(transform.position, coverPosition);
            Gizmos.DrawSphere(coverPosition, 2f);
        }
    }
}