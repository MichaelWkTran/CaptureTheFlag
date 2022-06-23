using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameController : MonoBehaviour
{
    [System.Serializable]
    public class Team
    {
        public List<Agent> Agents;
        public int Flags;
        public int AgentsCaptured;
        public Agent Attacker;
        public bool AttackerGoal; //If false, the attacker tries to capture a flag. If true, the attacker tries to save a agent

        [Header("[Game Variables]")]
        public GameObject FlagArea;
        public GameObject PrisonArea;

        [Header("[UI Variables]")]
        public TMPro.TextMeshProUGUI FlagCount;
        public TMPro.TextMeshProUGUI PlayerCount;
    }

    public Cinemachine.CinemachineVirtualCamera m_Camera;
    public Team m_PlayerTeam;
    public Team m_EnemyTeam;

    public int m_MaxFlags = 4;
    public int m_MaxAgents = 4;

    [Header("[Enemy Variables]")]
    public bool m_EnemyAttackerGoal = true; //True = Get flag, False = Get Agent
    public float m_MinEnemyAttackerTime;
    public float m_MaxEnemyAttackerTime;

    void Awake()
    {
        //Setup Player Team
        m_PlayerTeam.Flags = m_MaxFlags;
        m_PlayerTeam.Agents = new List<Agent>();

        //Setup Enemy Team
        m_EnemyTeam.Flags = m_MaxFlags;
        m_EnemyTeam.Agents = new List<Agent>();

        //--------------------
        StartCoroutine(ChooseEnemyAttacker());
    }

    void Update()
    {
        //Replace player team attacker if captured and set camera follow target
        if (m_PlayerTeam.Agents.Count > 0)
        {
            if (m_PlayerTeam.Attacker == null) m_PlayerTeam.Attacker = m_PlayerTeam.Agents[0];
            m_Camera.Follow = m_PlayerTeam.Attacker.transform;
        }

        //Update Player UI
        m_PlayerTeam.FlagCount.text = "Flags:" + m_PlayerTeam.Flags.ToString();
        m_PlayerTeam.PlayerCount.text = "Players Remaining:" + m_PlayerTeam.Agents.Count.ToString();

        //Update Enemy UI
        m_EnemyTeam.FlagCount.text = "Flags:" + m_EnemyTeam.Flags.ToString();
        m_EnemyTeam.PlayerCount.text = "Players Remaining:" + m_EnemyTeam.Agents.Count.ToString();
    }

    IEnumerator ChooseEnemyAttacker()
    {
        yield return new WaitForSeconds(Random.Range(m_MinEnemyAttackerTime, m_MaxEnemyAttackerTime));
        StartCoroutine(ChooseEnemyAttacker());

        if (m_EnemyTeam.Attacker == null)
        {
            float fGetFlagScore = Random.Range(0.0f, (m_EnemyTeam.Flags - m_MaxFlags) / m_MaxFlags);
            float fGetAgentScore = Random.Range(0.0f, (m_EnemyTeam.Agents.Count - m_MaxAgents) / m_MaxAgents);

            if (fGetFlagScore > fGetAgentScore) m_EnemyAttackerGoal = true;
            else m_EnemyAttackerGoal = false;

            m_EnemyTeam.Attacker = m_EnemyTeam.Agents[0];
        }
    }
}
