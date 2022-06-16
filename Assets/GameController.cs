using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
    [System.Serializable]
    public struct Team
    {
        public Agent[] Agents;
        public int Flags;
        public int AgentsCaptured;
        public Agent Attacker;
        public bool AttackerGoal; //If false, the attacker tries to capture a flag. If true, the attacker tries to save a agent

        [Header("[Game Variables]")]
        public GameObject FlagArea;
        public GameObject PrisonArea;

        [Header("[UI Variables]")]
        public TMPro.TextMeshProUGUI FlagCount;
        public TMPro.TextMeshProUGUI CapturedCount;
    }

    [SerializeField] Cinemachine.CinemachineVirtualCamera m_Camera;
    public Team m_PlayerTeam;
    public Team m_EnemyTeam;

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

            m_PlayerTeam.Attacker = m_PlayerTeam.Agents[0];
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
        m_Camera.Follow = m_PlayerTeam.Attacker.transform;

        m_PlayerTeam.FlagCount.text = "Flags:" + m_PlayerTeam.Flags.ToString();
        m_PlayerTeam.CapturedCount.text = "Captured:" + m_PlayerTeam.AgentsCaptured.ToString();
    }
}
