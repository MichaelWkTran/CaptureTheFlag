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

    [Header("[Game Variables]")]
    public bool m_OnOpponentSide = false;
    public bool m_HaveFlag = false;
    public bool m_SavedAgent = false;
    GameController m_GameController;

    [Header("[Movement Variables]")]
    [SerializeField] float m_MaxVelocity;
    public Vector2 m_Force;
    [SerializeField] float m_MaxForce;
    [SerializeField] float m_CurrentSpeed;

    [Header("[Steering Behaviour Variables]")]
    #region Steering Behaviour Variables
    [SerializeField] Vector2 m_SteeringTarget;
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
    Vector2 m_WanderRelativeTarget;
    
    [Header("Separation Variables")]
    [SerializeField] float m_SeparationDistance;
    [Header("Obstacle Avoidance Variables")]
    [SerializeField] float m_LocalSpaceWidth;
    [SerializeField] float m_ObstacleRadiusMultiply = 1;
    [Header("Wall Avoidance Variables")]
    [SerializeField] LayerMask m_WallLayerMask;
    [SerializeField] uint m_RayNumber = 8;
    [SerializeField] float m_WallSightLength;
    #endregion
    
    [Header("[State Machine Variables]")]
    [SerializeField] Unity.VisualScripting.StateMachine m_StateMachine;
    [Header("Default")]
    [SerializeField] float m_TeamBorderDistance;
    [Header("Chase")]
    [SerializeField] bool m_ChaseMethod = false; //False = Seek, True = Pursue
    
    [Header("[Component Variables]")]
    public Rigidbody2D m_RigidBody;

    public struct SteeringBehaviour
    {
        public delegate Vector2 SteeringDelegate();
        public SteeringDelegate Behaviour;
        public float Weight;

        public SteeringBehaviour(SteeringDelegate _Behaviour, float _Weight)
        {
            Behaviour = _Behaviour;
            Weight = _Weight;
        }
    }

    void Start()
    {
        m_GameController = FindObjectOfType<GameController>();
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
                if (Hit.transform.tag == "Player" && Hit.transform.gameObject == gameObject) m_GameController.m_PlayerTeam.Attacker = this;
            }
        }

        //Check whether on the opponent side
        if (tag == "Player")
        {
            //The Left side is the enemy side
            m_OnOpponentSide = (transform.position.x < 0) ? true : false;
        }
        else
        {
            //The right side is the player side
            m_OnOpponentSide = (transform.position.x > 0) ? true : false;
        }

        //Reset the agent when reaching to their side
        //If they have a flag, decrease the opponent flag count
        if (m_OnOpponentSide == false)
        {
            GameController.Team OpponentTeam = (tag == "Player") ? m_GameController.m_EnemyTeam : m_GameController.m_PlayerTeam;

            if (m_HaveFlag)
            {
                OpponentTeam.Flags--;
                m_HaveFlag = false;
            }
            else if(m_SavedAgent)
            {
                m_SavedAgent = false;
            }
        }
    }

    void FixedUpdate()
    {
        //Choose movement whether it is the player or not
        if (m_GameController.m_PlayerTeam.Attacker == this)
        {
            m_StateMachine.enabled = false;

            m_Force = SteeringBehaviours2D.Seek
            (
                transform.position, m_RigidBody.velocity, m_MaxVelocity,
                transform.position + (2.0f * m_MaxVelocity * new Vector3(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical")))
            );
        }
        else
        {
            m_StateMachine.enabled = true;
        }

        //Move the agent
        m_RigidBody.AddForce(Vector2.ClampMagnitude(m_Force, m_MaxForce));
        m_CurrentSpeed = m_RigidBody.velocity.magnitude;
        if (m_RigidBody.velocity != Vector2.zero)
        {
            transform.rotation = Quaternion.Euler(0.0f, 0.0f, (Mathf.Atan2(m_RigidBody.velocity.y, m_RigidBody.velocity.x) * Mathf.Rad2Deg) - 90.0f);
        }
    }

    #region States
    public void DefaultFixedUpdate()
    {
        m_ChaseMethod = Mathf.Round(Random.value) == 0;
        bool NearBorder = false;
        
        //Check whether on the opponent side
        if (tag == "Player")
        {
            //The Left side is the enemy side
            NearBorder = transform.position.x < m_TeamBorderDistance;
        }
        else
        {
            //The right side is the player side
            NearBorder = transform.position.x > -m_TeamBorderDistance;
        }

        if (!NearBorder)
        {
            Vector2 WallAvoidance()
            {
                return SteeringBehaviours2D.WallAvoidance
                (
                    transform.position, m_RigidBody.velocity, transform.up, m_MaxVelocity,
                    m_WallLayerMask, m_WallSightLength, m_RayNumber
                );
            }

            Vector2 Wander()
            {
                return SteeringBehaviours2D.Wander
                (
                    transform.position, m_RigidBody.velocity, m_MaxVelocity,
                    m_WanderRadius, m_WanderDistance, ref m_WanderTimer, m_WanderCooldown, ref m_WanderRelativeTarget
                );
            }

            Vector2 Sepparation()
            {
                Collider2D[] Colliders = Physics2D.OverlapCircleAll(transform.position, GetComponent<CircleCollider2D>().radius + m_SeparationDistance);
                List<Vector2> Neighbours = new List<Vector2>();

                foreach (Collider2D Collider in Colliders)
                {
                    if
                    (
                        Collider.gameObject == gameObject ||
                        Collider.GetComponent<Agent>() == null ||
                        Collider.tag != tag
                    )
                    {
                        continue;
                    }

                    Neighbours.Add(Collider.transform.position);
                }

                if (Neighbours.Count <= 0) return Vector2.zero;

                return SteeringBehaviours2D.Sepparation(transform.position, m_RigidBody.velocity, m_MaxVelocity, Neighbours.ToArray());
            }

            SteeringBehaviours2D.BehaviourDelegate[] SteeringBehaviours = new SteeringBehaviours2D.BehaviourDelegate[3]
                {WallAvoidance, Sepparation, Wander};

            m_Force = SteeringBehaviours2D.WeightedTruncatedRunningSumWithPrioritization(SteeringBehaviours, m_MaxForce);
        }
        else
        {
            Vector2 ReturnPoint = new Vector2(-100, 0);
            if (tag == "Player") ReturnPoint = new Vector2(100, 0);

            m_Force = SteeringBehaviours2D.Seek(transform.position, m_RigidBody.velocity, m_MaxVelocity, ReturnPoint);
        }
    }

    public void ChaseFixedUpdate()
    {
        Agent AttackingAgent = (tag == "Player") ? m_GameController.m_EnemyTeam.Attacker : m_GameController.m_PlayerTeam.Attacker;
        if (AttackingAgent == null) return;
        if (!AttackingAgent.m_OnOpponentSide) return;

        SteeringBehaviours2D.BehaviourDelegate[] SteeringBehaviours = new SteeringBehaviours2D.BehaviourDelegate[2];

        if (m_ChaseMethod)
        {
            Vector2 Pursue()
            {
                return SteeringBehaviours2D.Pursue
                (
                    transform.position, m_RigidBody.velocity, m_MaxVelocity,
                    m_MaxPrediction,
                    AttackingAgent.transform.position,
                    AttackingAgent.m_RigidBody.velocity
                );  
            }
            SteeringBehaviours[0] = Pursue;
        }
        else
        {
            Vector2 Seek()
            {
                return SteeringBehaviours2D.Seek(transform.position, m_RigidBody.velocity, m_MaxVelocity, AttackingAgent.transform.position);
            }
            SteeringBehaviours[0] = Seek;
        }

        Vector2 Sepparation()
        {
            Collider2D[] Colliders = Physics2D.OverlapCircleAll(transform.position, GetComponent<CircleCollider2D>().radius + m_SeparationDistance);
            List<Vector2> Neighbours = new List<Vector2>();

            foreach (Collider2D Collider in Colliders)
            {
                if
                (
                    Collider.gameObject == gameObject ||
                    Collider.GetComponent<Agent>() == null ||
                    Collider.tag != tag
                )
                {
                    continue;
                }

                Neighbours.Add(Collider.transform.position);
            }

            if (Neighbours.Count <= 0) return Vector2.zero;

            return SteeringBehaviours2D.Sepparation(transform.position, m_RigidBody.velocity, m_MaxVelocity, Neighbours.ToArray());
        }
        SteeringBehaviours[1] = Sepparation;

        m_Force = SteeringBehaviours2D.WeightedTruncatedRunningSumWithPrioritization(SteeringBehaviours, m_MaxForce);
    }

    public void AttackFixedUpdate()
    {

    }

    #endregion States

    #region Transitions
    public bool DefaultToChaseUpdate()
    {
        GameController.Team OpponentTeam = (tag == "Player") ? m_GameController.m_EnemyTeam : m_GameController.m_PlayerTeam;

        if (OpponentTeam.Attacker == null) return false;
        if (OpponentTeam.Attacker.m_OnOpponentSide) return true;

        return false;
    }

    public bool ChaseToDefaultUpdate()
    {
        GameController.Team OpponentTeam = (tag == "Player") ? m_GameController.m_EnemyTeam : m_GameController.m_PlayerTeam;
        if (OpponentTeam.Attacker == null) return true;
        if (OpponentTeam.Attacker.m_OnOpponentSide) return false;

        return true;
    }

    #endregion Transitions

    void OnDrawGizmosSelected()
    {
        switch (m_DrawSteeringBehaviour)
        {
            case DrawSteeringBehaviour.Arrive:
            {
                Gizmos.DrawWireSphere(m_SteeringTarget, m_SlowingRadius);
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
                Gizmos.DrawWireSphere(m_SteeringTarget, 0.5f);
                break;
            }
            case DrawSteeringBehaviour.Separation:
            {
                Gizmos.DrawWireSphere(transform.position, m_SeparationDistance + GetComponent<CircleCollider2D>().radius);
                break;
            }
            case DrawSteeringBehaviour.ObstacleAvoidance:
            {
                for (uint i = 0; i < m_RayNumber; i++)
                {
                    Gizmos.DrawLine
                    (
                        transform.position,
                        transform.position + 
                        (
                            Quaternion.Euler(0.0f, 0.0f, i * 360.0f / m_RayNumber) * 
                            transform.up * 
                            m_WallSightLength
                        )
                    );
                }


                break;
            }
        }
    }
}