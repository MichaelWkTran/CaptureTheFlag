namespace FiniteStateMachineNamespace
{
    public class FiniteStateMachine
    {
        public Node[] m_AvaliableNodes;

        public Node m_CurrentNode;
        public Node m_PreviousNode;

        public FiniteStateMachine(Node[] _AvaliableNodes)
        {
            m_AvaliableNodes = _AvaliableNodes;

            foreach (Node Node in m_AvaliableNodes) Node.m_FinateStateMachine = this;

            m_CurrentNode = _AvaliableNodes[0];
        }

        public Node FindNodeWithName(string _Name)
        {
            foreach (Node Node in m_AvaliableNodes) if (Node.m_Name == _Name) return Node;

            return null;
        }

        public void Update()
        {
            if (m_CurrentNode != null && m_CurrentNode.m_UpdateFunctions != null) m_CurrentNode.Update();
        }

        public void FixedUpdate()
        {
            if (m_CurrentNode != null && m_CurrentNode.m_FixedUpdateFunctions != null) m_CurrentNode.FixedUpdate();
        }

        public void LateUpdate()
        {
            if (m_CurrentNode != null && m_CurrentNode.m_LateUpdateFunctions != null) m_CurrentNode.LateUpdate();
        }
    }

    public class Node
    {
        //Must always be created with a FiniteStateMachine

        public string m_Name;
        public FiniteStateMachine m_FinateStateMachine;

        public delegate void NodeBehaviour(Node _ThisNode);
        public NodeBehaviour m_UpdateFunctions;
        public NodeBehaviour m_FixedUpdateFunctions;
        public NodeBehaviour m_LateUpdateFunctions;

        public Node(string _Name, NodeBehaviour _UpdateFunctions = null, NodeBehaviour _FixedUpdateFunctions = null)
        {
            m_Name = _Name;
            m_UpdateFunctions = _UpdateFunctions;
            m_FixedUpdateFunctions = _FixedUpdateFunctions;
            m_LateUpdateFunctions = _FixedUpdateFunctions;
        }

        public void Update()
        {
            if (m_FinateStateMachine != null && m_UpdateFunctions != null) m_UpdateFunctions(this);
        }

        public void FixedUpdate()
        {
            if (m_FinateStateMachine != null && m_FixedUpdateFunctions != null) m_FixedUpdateFunctions(this);
        }

        public void LateUpdate()
        {
            if (m_FinateStateMachine != null && m_LateUpdateFunctions != null) m_LateUpdateFunctions(this);
        }
    }
}