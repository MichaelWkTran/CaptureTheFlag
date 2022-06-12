using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Pathfinding;

public class Agent : MonoBehaviour
{
    enum DrawSteeringBehaviour
    {
        None,
        Arrive,
        PursueEvade,
        Wander,
        Separation,
        ObstacleAvoidance
    };

    [Header("[Game Variables]")]
    [SerializeField] bool m_HaveFlag = false;
    [SerializeField] bool m_SavedAgent = false;
    GameController m_GameController;

    [Header("[Movement Variables]")]
    [SerializeField] float m_MaxVelocity;
    public Vector2 m_Force;
    [SerializeField] float m_MaxForce;
    [SerializeField] float m_CurrentSpeed;


    [Header("[Steering Behaviour Variables]")]
    [SerializeField] Unity.VisualScripting.StateMachine m_StateMachine;
    [SerializeField] Vector2 m_Target;
    [SerializeField] Agent m_AgentTarget;
    public delegate Vector2 BehaviourDelegate();
    List<BehaviourDelegate> m_ActiveSteeringBehaviours = new List<BehaviourDelegate>();
    [SerializeField] DrawSteeringBehaviour m_DrawSteeringBehaviour;

    [Header("Arrive Variables")]
    [SerializeField] float m_SlowingRadius;
    [Header("Pursue/Evade Variables")]
    [SerializeField] float m_MaxPrediction;
    [Header("Wander Variables")]
    [SerializeField] float m_WanderTimer;
    [SerializeField] float m_WanderCooldown;
    [SerializeField] float m_WanderDistance;
    [SerializeField] float m_WanderRadius;
    Vector2 m_WanderTarget;
    [Header("Separation Variables")]
    [SerializeField] float m_SeparationDistance;
    [Header("Obstacle Avoidance Variables")]
    [SerializeField] float m_LocalSpaceWidth;
    [SerializeField] float m_ObstacleRadiusMultiply = 1;

    [Header("[Pathfinding Variables]")]
    public float NextWaypointDistance = 3.0f;
    [SerializeField] Path path;
    int currentWaypoint;
    bool reached;

    public Seeker seeker;
    public Rigidbody2D rb;

    void Start()
    {
        m_GameController = FindObjectOfType<GameController>();
        seeker.StartPath(transform.position, m_Target, OnPathComplete);
    }

    void OnPathComplete(Path p)
    {
        if (!p.error)
        {
            path = p;
            currentWaypoint = 0;
        }
    }

    void Update()
    {
        //Ensure the agent z position is locked at 0
        if (transform.position.z != 0) transform.position = new Vector2(transform.position.x, transform.position.y);

        //Click agent to make it the player
        if (Input.GetMouseButtonDown(0))
        {
            RaycastHit2D Hit = Physics2D.Raycast(Camera.main.ScreenToWorldPoint(Input.mousePosition), Vector2.zero);
            if (Hit.collider != null)
            {
                if (Hit.transform.tag == "Player" && Hit.transform.gameObject == gameObject) m_GameController.m_Player = this;
            }
        }

        //Reset the agent when reaching to their side
        if (tag == "Player")
        {
            if (transform.position.x > 0)
            {
                if (m_HaveFlag)
                {
                    m_GameController.m_EnemyTeam.Flags--;
                    m_HaveFlag = false;
                }
                else if (m_SavedAgent)
                {
                    m_SavedAgent = false;
                }
            }
        }
        else
        {
            if (transform.position.x < 0)
            {
                if (m_HaveFlag)
                {
                    m_GameController.m_PlayerTeam.Flags--;
                    m_HaveFlag = false;
                }
                else if (m_SavedAgent)
                {
                    m_SavedAgent = false;
                }
            }
        }
    }

    void FixedUpdate()
    {
        //Choose movement whether it is the player or not
        if (m_GameController.m_Player == this)
        {
            m_StateMachine.enabled = false;

            m_Target = transform.position + (2.0f * m_MaxVelocity * new Vector3(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical")));
            m_Force = Seek();
        }
        else
        {
            m_StateMachine.enabled = true;
        }

        //Move the agent
        rb.AddForce(Truncate(m_Force, m_MaxForce));
        m_CurrentSpeed = rb.velocity.magnitude;
        if (rb.velocity != Vector2.zero)
        {
            transform.rotation = Quaternion.Euler(0.0f, 0.0f, (Mathf.Atan2(rb.velocity.y, rb.velocity.x) * Mathf.Rad2Deg) - 90.0f);
        }
    }

    Vector2 Truncate(Vector2 _Vector, float _MaxMagnitude)
    {
        if (_Vector.magnitude > _MaxMagnitude) return _Vector.normalized * _MaxMagnitude;
        else return _Vector;
    }

    #region Steering Behaviours
    Vector2 Seek()
    {
        Vector2 DesiredVelocity = (m_Target - new Vector2(transform.position.x, transform.position.y)).normalized * m_MaxVelocity;
        return DesiredVelocity - rb.velocity;
    }

    Vector2 Flee()
    {
        Vector2 DesiredVelocity = (new Vector2(transform.position.x, transform.position.y) - m_Target).normalized * m_MaxVelocity;
        return DesiredVelocity - rb.velocity;
    }

    Vector2 Pursue()
    {
        float Speed = rb.velocity.magnitude;
        float Distance = Vector2.Distance(m_AgentTarget.transform.position, transform.position);

        float Prediction = (Speed <= Distance / m_MaxPrediction) ? m_MaxPrediction : Distance / Speed;

        m_Target = new Vector2(m_AgentTarget.transform.position.x, m_AgentTarget.transform.position.y) + (m_AgentTarget.rb.velocity * Prediction);
        return Seek();
    }

    Vector2 Evade()
    {
        float Speed = rb.velocity.magnitude;
        float Distance = Vector2.Distance(m_AgentTarget.transform.position, transform.position);

        float Prediction = (Speed <= Distance / m_MaxPrediction) ? m_MaxPrediction : Distance / Speed;

        m_Target = new Vector2(m_AgentTarget.transform.position.x, m_AgentTarget.transform.position.y) + (m_AgentTarget.rb.velocity * Prediction);
        return Flee();
    }

    Vector2 Arrive()
    {
        Vector2 TargetOffset = new Vector3(m_Target.x, m_Target.y) - transform.position;
        float Distance = TargetOffset.magnitude;

        if (Distance < m_SlowingRadius)
        {
            Vector2 v2fDesiredVelocity = TargetOffset.normalized * m_MaxVelocity * (Distance / m_SlowingRadius);
            return v2fDesiredVelocity - rb.velocity;
        }

        return Seek();
    }

    Vector2 Wander()
    {
        if (m_WanderTimer <= 0)
        {
            float WanderAngle = Random.Range(0.0f, 360.0f) * Mathf.Deg2Rad;
            m_WanderTarget = m_WanderRadius * new Vector2(Mathf.Cos(WanderAngle), Mathf.Sin(WanderAngle));
            m_WanderTarget += rb.velocity.normalized * m_WanderDistance;

            m_WanderTimer = m_WanderCooldown;
        }
        else
        {
            m_WanderTimer -= Time.deltaTime;
        }

        m_Target = m_WanderTarget + new Vector2(transform.position.x, transform.position.y);

        return Seek();
    }

    Vector2 Sepparation()
    {
        Collider2D[] Neighbours = Physics2D.OverlapCircleAll(transform.position, GetComponent<CircleCollider2D>().radius + m_SeparationDistance);
        if (Neighbours.Length <= 0) return Vector2.zero;

        Vector2 DesiredVelocity = Vector2.zero;

        int NeighbourCount = 0;
        foreach (Collider2D Neighbour in Neighbours)
        {
            if (Neighbour.GetComponent<Agent>() == null) continue;
            NeighbourCount++;

            Vector2 Difference = new Vector2(transform.position.x, transform.position.y) - new Vector2(Neighbour.transform.position.x, Neighbour.transform.position.y);
            if (Difference == Vector2.zero) continue;

            DesiredVelocity += Difference.normalized / Difference.magnitude;
        }
        DesiredVelocity /= NeighbourCount;
        DesiredVelocity = DesiredVelocity.normalized * m_MaxVelocity;

        return DesiredVelocity - rb.velocity;
    }

    Vector2 ObjectAvoidance()
    {
        //Find agents within current speed
        Collider2D[] Neighbours = Physics2D.OverlapCircleAll(transform.position, rb.velocity.magnitude);
        
        //---------------------------------------
        Vector3 PriorityNeibourPosition = Vector3.zero;
        float PriorityNeibourRadius = 0;
        float PriorityNeibourDistance = Mathf.Infinity;
        
        foreach (Collider2D Neighbour in Neighbours)
        {
            //Ignore checking this agent
            if (Neighbour.GetComponent<Agent>() == null) continue;
            //Ignore any obstacles behind the agent
            if (Vector3.Dot((Neighbour.transform.position - transform.position).normalized, transform.up) <= 0) continue;

            //Expand Obstacle Radius
            float NeighbourRadius = Neighbour.GetComponent<CircleCollider2D>().radius + (m_LocalSpaceWidth / 2.0f);
            NeighbourRadius *= m_ObstacleRadiusMultiply;

            //Find intersections
            //https://answers.unity.com/questions/1658184/circle-line-intersection-points.html
            Vector2 Intersect1, Intersect2;
            Vector2 LineSegmentStart = transform.position;
            Vector2 LineSegmentEnd = transform.position + new Vector3(rb.velocity.x, rb.velocity.y);

            Vector2 dp = LineSegmentEnd - LineSegmentStart;

            float a = Vector2.Dot(dp, dp);
            float b = 2 * Vector2.Dot(dp, LineSegmentStart - new Vector2(Neighbour.transform.position.x, Neighbour.transform.position.y));
            float c = Vector2.Dot(Neighbour.transform.position, Neighbour.transform.position)
                      - 2 * Vector2.Dot(Neighbour.transform.position, LineSegmentStart) + 
                      Vector2.Dot(LineSegmentStart, LineSegmentStart) - NeighbourRadius * NeighbourRadius;
            float bb4ac = b * b - 4 * a * c;
            if (Mathf.Abs(a) < float.Epsilon || bb4ac < 0)
            {
                //line does not intersect
                continue;
            }
            float mu1 = (-b + Mathf.Sqrt(bb4ac)) / (2 * a);
            float mu2 = (-b - Mathf.Sqrt(bb4ac)) / (2 * a);
            Vector2[] sect = new Vector2[2];

            Intersect1 = LineSegmentStart + mu1 * (LineSegmentEnd - LineSegmentStart);
            Intersect2 = LineSegmentStart + mu2 * (LineSegmentEnd - LineSegmentStart);

            //Compare intersections
            if (Vector2.Distance(transform.position, Intersect1) < PriorityNeibourDistance)
            {
                PriorityNeibourPosition = Intersect1;
                PriorityNeibourRadius = NeighbourRadius;
                PriorityNeibourDistance = Vector2.Distance(transform.position, Intersect1);
            }
        
            if (Vector2.Distance(transform.position, Intersect2) < PriorityNeibourDistance)
            {
                PriorityNeibourPosition = Intersect2;
                PriorityNeibourRadius = NeighbourRadius;
                PriorityNeibourDistance = Vector2.Distance(transform.position, Intersect2);
            }
        }
        
        //---------------------------------------------
        if (PriorityNeibourDistance == Mathf.Infinity) return Vector2.zero;

        //--------------------------------
        return (transform.up * -PriorityNeibourPosition.magnitude) +
               (transform.right * (PriorityNeibourRadius - PriorityNeibourPosition.y) * PriorityNeibourPosition.magnitude);
    }
    #endregion Steering Behaviours

    #region States
    public void DefaultEnter()
    {
        m_Force = Sepparation();
        m_Force += Wander() * ((m_MaxForce - m_Force.magnitude) / m_MaxForce);
    }

    #endregion States

    void OnDrawGizmosSelected()
    {
        switch (m_DrawSteeringBehaviour)
        {
            case DrawSteeringBehaviour.Arrive:
            {
                Gizmos.DrawWireSphere(m_Target, m_SlowingRadius);
                break;
            }
            case DrawSteeringBehaviour.PursueEvade:
            {
                Gizmos.DrawLine(transform.position, transform.position + (transform.up * m_MaxPrediction));
                break;
            }
            case DrawSteeringBehaviour.Wander:
            {
                Vector3 WanderTarget = transform.position + (transform.up * m_WanderDistance);
                Gizmos.DrawWireSphere(WanderTarget, m_WanderRadius);

                Gizmos.color = Color.blue;
                Gizmos.DrawWireSphere(m_Target, 0.5f);
                break;
            }
            case DrawSteeringBehaviour.Separation:
            {
                Gizmos.DrawWireSphere(transform.position, m_SeparationDistance + GetComponent<CircleCollider2D>().radius);
                break;
            }
            case DrawSteeringBehaviour.ObstacleAvoidance:
            {

                break;
            }
        }
    }
}

/*
if (path == null) return; 

if(currentWaypoint >= path.vectorPath.Count) { reachedEndOfPath = true; return; }
else { reachedEndOfPath = false; }

Vector2 direction ((Vector2)path.vectorPath[currentWaypoint] - rb.position).normalized;
Vector2 force = direction * speed * Time.deltaTime;

rb.AddForce(force);

float distance = Vector2.Distance(rb.position, path.vectorPath[currentWaypoint]);

if (distance < nextWayPointDistance)
{
	currentWaypoint;
}
 */