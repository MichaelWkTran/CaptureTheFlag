using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
    [SerializeField] Agent[] m_PlayerAgents;
    [SerializeField] Agent[] m_EnemyAgents;
    [SerializeField] Cinemachine.CinemachineVirtualCamera m_Camera;
    static public Agent m_Player;



    // Start is called before the first frame update
    void Start()
    {
        //Find Players
        {
            GameObject[] Players = GameObject.FindGameObjectsWithTag("Player");
            m_PlayerAgents = new Agent[Players.Length];
            
            int PlayerIndex = 0;
            foreach(GameObject Player in Players)
            {
                m_PlayerAgents[PlayerIndex] = Player.GetComponent<Agent>();
                PlayerIndex++;
            }

            m_Player = m_PlayerAgents[0];
        }

        //Find Enemies
        {
            GameObject[] Enemies = GameObject.FindGameObjectsWithTag("Enemy");
            m_EnemyAgents = new Agent[Enemies.Length];

            int EnemyIndex = 0;
            foreach (GameObject Enemy in Enemies)
            {
                m_EnemyAgents[EnemyIndex] = Enemy.GetComponent<Agent>();
                EnemyIndex++;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        m_Camera.Follow = m_Player.transform;
    }
}
