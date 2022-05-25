using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    public float m_Mass = 1;
    public Vector2 m_Velocity;
    [SerializeField] float m_MaxVelocity;
    public Vector2 m_Force;
    [SerializeField] float m_MaxForce;

    [Header("[Steering Behaviour Variables]")]
    [SerializeField] Unity.VisualScripting.StateMachine m_StateMachine;
    [SerializeField] Vector2 m_Target;
    [SerializeField] Agent m_AgentTarget;
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
    [Header("Separation Variables")]
    [SerializeField] float m_SeparationDistance;
    [Header("Obstacle Avoidance Variables")]
    [SerializeField] float m_LocalSpaceWidth;
    [SerializeField] float m_ObstacleRadiusMultiply = 1;

    void Start()
    {
        
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
                if (Hit.transform.tag == "Player" && Hit.transform.gameObject == gameObject) GameController.m_Player = this;
            }
        }

        //Choose movement whether it is the player or not
        if (GameController.m_Player == this)
        {
            m_StateMachine.enabled = false;
            m_Target = transform.position + (100.0f * new Vector3(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical")));
            m_Force = Arrive();
        }
        else
        {
            m_StateMachine.enabled = true;
        }

        //Move the agent
        m_Velocity += (Truncate(m_Force, m_MaxForce) / m_Mass) * Time.deltaTime;
        transform.position += new Vector3(m_Velocity.x, m_Velocity.y) * Time.deltaTime;
        transform.rotation = Quaternion.Euler(0.0f, 0.0f, (Mathf.Atan2(m_Velocity.y, m_Velocity.x) * Mathf.Rad2Deg) - 90.0f);
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
        return DesiredVelocity - m_Velocity;
    }

    Vector2 Flee()
    {
        Vector2 DesiredVelocity = (new Vector2(transform.position.x, transform.position.y) - m_Target).normalized * m_MaxVelocity;
        return DesiredVelocity - m_Velocity;
    }

    Vector2 Pursue()
    {
        float Speed = m_Velocity.magnitude;
        float Distance = Vector2.Distance(m_AgentTarget.transform.position, transform.position);

        float Prediction = (Speed <= Distance / m_MaxPrediction) ? m_MaxPrediction : Distance / Speed;

        m_Target = new Vector2(m_AgentTarget.transform.position.x, m_AgentTarget.transform.position.y) + (m_AgentTarget.m_Velocity * Prediction);
        return Seek();
    }

    Vector2 Evade()
    {
        float Speed = m_Velocity.magnitude;
        float Distance = Vector2.Distance(m_AgentTarget.transform.position, transform.position);

        float Prediction = (Speed <= Distance / m_MaxPrediction) ? m_MaxPrediction : Distance / Speed;

        m_Target = new Vector2(m_AgentTarget.transform.position.x, m_AgentTarget.transform.position.y) + (m_AgentTarget.m_Velocity * Prediction);
        return Flee();
    }

    Vector2 Arrive()
    {
        Vector2 TargetOffset = new Vector3(m_Target.x, m_Target.y) - transform.position;
        float Distance = TargetOffset.magnitude;

        if (Distance < m_SlowingRadius)
        {
            Vector2 v2fDesiredVelocity = TargetOffset.normalized * m_MaxVelocity * (Distance / m_SlowingRadius);
            return v2fDesiredVelocity - m_Velocity;
        }

        return Seek();
    }

    Vector2 Wander()
    {
        float WanderAngle = Random.Range(0.0f, 360.0f) * Mathf.Deg2Rad;
        m_Target = new Vector2(Mathf.Cos(WanderAngle), Mathf.Sin(WanderAngle));
        m_Target *= m_WanderRadius;
        m_Target += new Vector2(transform.position.x, transform.position.y) + (m_Velocity.normalized * m_WanderDistance);

        return Seek();
    }

    Vector2 Sepparation()
    {
        Vector2 DesiredVelocity = Vector2.zero;

        Collider2D[] Neighbours = Physics2D.OverlapCircleAll(transform.position, GetComponent<CircleCollider2D>().radius + m_SeparationDistance);

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

        return DesiredVelocity - m_Velocity;
    }

    Vector2 CircleObjectAvoidance()
    {
        //Find agents within current speed
        Collider2D[] Neighbours = Physics2D.OverlapCircleAll(transform.position, m_Velocity.magnitude);

        //---------------------------------------
        Vector3 PriorityNeibourPosition = Vector3.zero;
        float PriorityNeibourRadius = 0;
        float PriorityNeibourIntersection = Mathf.Infinity;

        foreach (Collider2D Neighbour in Neighbours)
        {
            if (Neighbour.GetComponent<Agent>() == null) continue;

            //Convert to local space
            Vector3 NeighbourLocalPosition = Quaternion.Inverse(transform.rotation) * (Neighbour.transform.position - transform.position);

            //Discard neighbours behind the agent
            if (NeighbourLocalPosition.x < 0) continue;

            //Expand Radius
            float NeighbourRadius = GetComponent<CircleCollider2D>().radius + (m_LocalSpaceWidth / 2.0f);
            NeighbourRadius *= m_ObstacleRadiusMultiply;

            //Discard neighbours to far away to be detected
            if (Mathf.Abs(NeighbourLocalPosition.y) > NeighbourRadius) continue;

            //Find and compare intersections
                //fB and fC represent the variables in the quadratic formula used to calculate the circle intersection with the y-axis
            float fB = -2.0f * NeighbourLocalPosition.x;
            float fC = (NeighbourLocalPosition.x * NeighbourLocalPosition.x) - (NeighbourRadius * NeighbourRadius) + (NeighbourLocalPosition.y * NeighbourLocalPosition.y);
            
            float fIntersection = (-fB + Mathf.Sqrt((fB * fB) - (4 * fC))) / 2.0f;
            if (fIntersection < PriorityNeibourIntersection)
            {
                PriorityNeibourPosition = NeighbourLocalPosition;
                PriorityNeibourRadius = NeighbourRadius;
                PriorityNeibourIntersection = fIntersection;
            }

            fIntersection = (-fB - Mathf.Sqrt((fB * fB) - (4 * fC))) / 2.0f;
            if (fIntersection < PriorityNeibourIntersection)
            {
                PriorityNeibourPosition = NeighbourLocalPosition;
                PriorityNeibourRadius = NeighbourRadius;
                PriorityNeibourIntersection = fIntersection;
            }
        }

        //---------------------------------------------
        if (PriorityNeibourIntersection == Mathf.Infinity) return Vector2.zero;

        //--------------------------------
        Vector2 v2fForce = new Vector2
        (
            -PriorityNeibourPosition.magnitude, //BrakingForce
			(PriorityNeibourRadius - PriorityNeibourPosition.y) * PriorityNeibourPosition.magnitude //Lateral Force
		);
        v2fForce = transform.rotation * v2fForce;

        return v2fForce;
    }
    #endregion Steering Behaviours

    #region States

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

        //[Header("Separation Variables")]
        //[SerializeField] float m_SeparationDistance;
        //[Header("Obstacle Avoidance Variables")]
        //[SerializeField] float m_LocalSpaceWidth;
        //[SerializeField] float m_ObstacleRadiusMultiply;
    }
}