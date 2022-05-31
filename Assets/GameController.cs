using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
    public struct Team
    {
        public Agent[] Agents;
        public int Flags;
    }

    [SerializeField] Cinemachine.CinemachineVirtualCamera m_Camera;
    public Team m_PlayerTeam;
    public Team m_EnemyTeam;
    public Agent m_Player;

    [SerializeField] int m_MaxFlags = 3;

    void Start()
    {
        //Setup Player Team
        {
            m_PlayerTeam.Flags = m_MaxFlags;

            //Find Players
            GameObject[] Players = GameObject.FindGameObjectsWithTag("Player");
            m_PlayerTeam.Agents = new Agent[Players.Length];
            
            int PlayerIndex = 0;
            foreach(GameObject Player in Players)
            {
                m_PlayerTeam.Agents[PlayerIndex] = Player.GetComponent<Agent>();
                PlayerIndex++;
            }

            m_Player = m_PlayerTeam.Agents[0];
        }

        //Setup Enemy Team
        {
            m_EnemyTeam.Flags = m_MaxFlags;

            //Find Enemies
            GameObject[] Enemies = GameObject.FindGameObjectsWithTag("Enemy");
            m_EnemyTeam.Agents = new Agent[Enemies.Length];

            int EnemyIndex = 0;
            foreach (GameObject Enemy in Enemies)
            {
                m_EnemyTeam.Agents[EnemyIndex] = Enemy.GetComponent<Agent>();
                EnemyIndex++;
            }
        }
    }

    void Update()
    {
        m_Camera.Follow = m_Player.transform;
    }
}
